# dependencies for retargeter
import time
import torch
import numpy as np
import os
import pytorch_kinematics as pk
from typing import Union
import yaml
from scipy.spatial.transform import Rotation

from dextrous_hand.mano import retarget_utils
from dextrous_hand.utils.constants import RETARGETER_PARAMS

class Retargeter:
    """
    Please note that the computed joint angles of the rolling joints are only half of the two joints combined.
    hand_scheme either a string of the yaml path or a dictionary of the hand scheme
    mano_adjustments is a dictionary of the adjustments to the mano joints.
        keys: "thumb", "index", "middle", "ring", "pinky"
        value is a dictionary with the following keys:
            translation: (3,) translation vector in palm frame
            rotation: (3) x,y,z angles in palm frame, around the finger base
            scale: (3,) scale factors in finger_base frame
    retargeter_cfg is a dictionary of the retargeter algorithm. Including the following options:
        lr: learning rate
        use_scalar_distance_palm: whether to use scalar distance for palm
        loss_coeffs: (5,) loss coefficients for each fingertip
        joint_regularizers: tuples (joint_name, zero_value, weight) for regularizing joints to zero
    """

    def __init__(
        self,
        urdf_filepath: str | None = None,
        mjcf_filepath: str | None = None,
        sdf_filepath: str | None = None,
        hand_scheme:  Union[str, dict] | None = None,
    ) -> None:
        assert (
            int(urdf_filepath is not None)
            + int(mjcf_filepath is not None)
            + int(sdf_filepath is not None)
        ) == 1, "Exactly one of urdf_filepath, mjcf_filepath, or sdf_filepath should be provided"


        if hand_scheme == "hh":
            from .hand_cfgs.hh_cfg import (
                GC_TENDONS,
                FINGER_TO_TIP,
                FINGER_TO_BASE,
                FINGER_TO_MP,
                GC_LIMITS_LOWER,
                GC_LIMITS_UPPER,
                WRIST_NAME
            )
        elif hand_scheme == "p4":
            from .hand_cfgs.p4_cfg import (
                GC_TENDONS,
                FINGER_TO_TIP,
                FINGER_TO_BASE,
                GC_LIMITS_LOWER,
                GC_LIMITS_UPPER,
            )
        else:
            raise ValueError(f"hand_model {hand_scheme} not supported")

        self.mano_adjustments = RETARGETER_PARAMS["retargeter_adjustments"]

        self.device = RETARGETER_PARAMS["device"]

        self.lr = RETARGETER_PARAMS["lr"]
        self.use_scalar_distance_palm = RETARGETER_PARAMS["use_scalar_distance_palm"]

        self.joint_regularizers = RETARGETER_PARAMS["joint_regularizers"]

        self.target_angles = None

        self.gc_limits_lower = GC_LIMITS_LOWER
        self.gc_limits_upper = GC_LIMITS_UPPER
        self.finger_to_tip = FINGER_TO_TIP
        self.finger_to_base = FINGER_TO_BASE
        self.finger_to_mp = FINGER_TO_MP

        prev_cwd = os.getcwd()
        model_path = (
            urdf_filepath
            if urdf_filepath is not None
            else mjcf_filepath if mjcf_filepath is not None else sdf_filepath
        )
        if model_path is None:
            raise ValueError("No model file provided")

        model_dir_path = os.path.dirname(model_path)
        os.chdir(model_dir_path)
        if urdf_filepath is not None:
            self.chain = pk.build_chain_from_urdf(open(urdf_filepath).read()).to(
                device=self.device
            )
        elif mjcf_filepath is not None:
            self.chain = pk.build_chain_from_mjcf(open(mjcf_filepath).read()).to(
                device=self.device
            )
        elif sdf_filepath is not None:
            self.chain = pk.build_chain_from_sdf(open(sdf_filepath).read()).to(
                device=self.device
            )
        os.chdir(prev_cwd)

        ## This part builds the `joint_map` (n_joints, n_tendons) which is jacobian matrix.
        ## each tendon is a group of coupled joints that are driven by a single motor
        ## The rolling contact joints are modeled as a pair of joint and virtual joint
        ## The virtual joints are identified by the suffix "_virt"
        ## So, the output of the virtual joint will be the sum of the joint and its virtual counterpart, i.e. twice
        joint_parameter_names = self.chain.get_joint_parameter_names()
        gc_tendons = GC_TENDONS
        self.n_joints = self.chain.n_joints
        self.n_tendons = len(
            GC_TENDONS
        )  # each tendon can be understand as the tendon drive by a motor individually
        self.joint_map = torch.zeros(self.n_joints, self.n_tendons).to(self.device)
        self.finger_to_tip = FINGER_TO_TIP
        self.tendon_names = []
        joint_names_check = []
        for i, (name, tendons) in enumerate(gc_tendons.items()):
            virtual_joint_weight = 0.5 if name.endswith("_virt") else 1.0
            self.joint_map[joint_parameter_names.index(name), i] = virtual_joint_weight
            self.tendon_names.append(name)
            joint_names_check.append(name)
            for tendon, weight in tendons.items():
                self.joint_map[joint_parameter_names.index(tendon), i] = (
                    weight * virtual_joint_weight
                )
                joint_names_check.append(tendon)
        assert set(joint_names_check) == set(
            joint_parameter_names
        ), "Joint names mismatch, please double check hand_scheme"

        if hand_scheme == "hh":
            self.motor_count = 16
        else:
            self.motor_count = 15

        self.gc_joints = torch.ones(self.n_tendons).to(self.device) * float(self.motor_count)
        self.gc_joints.requires_grad_()

        self.regularizer_zeros = torch.zeros(self.n_tendons).to(self.device)
        self.regularizer_weights = torch.zeros(self.n_tendons).to(self.device)
        for joint_name, zero_value, weight in self.joint_regularizers:
            self.regularizer_zeros[self.tendon_names.index(joint_name)] = zero_value
            self.regularizer_weights[self.tendon_names.index(joint_name)] = weight

        # self.opt = torch.optim.Adam([self.gc_joints], lr=self.lr)
        optimizer =  RETARGETER_PARAMS["optimizer"]
        if optimizer == "Adam":
            self.opt = torch.optim.Adam([self.gc_joints], lr=self.lr)
        elif optimizer == "RMSprop":
            self.opt = torch.optim.RMSprop([self.gc_joints], lr=self.lr)
        else:
            raise ValueError("Optimizer " + optimizer + " not supported. Rewrite code to support it")

        self.root = torch.zeros(1, 3).to(self.device)

        self.loss_coeffs = torch.tensor(list(RETARGETER_PARAMS["loss_coeffs"].values())).to(self.device)

        if self.use_scalar_distance_palm:
            self.use_scalar_distance = [False] + [True] * (len(RETARGETER_PARAMS["keyvectors"]) - 1)
        else:
            self.use_scalar_distance = [False] * len(RETARGETER_PARAMS["keyvectors"])

        self.scale_coeffs = torch.tensor(list(RETARGETER_PARAMS["scale_coeffs"].values())).to(self.device)

        self.sanity_check()
        _chain_transforms = self.chain.forward_kinematics(
            torch.zeros(self.chain.n_joints, device=self.chain.device)
        )

        self.model_center, self.model_rotation = (
            retarget_utils.get_hand_center_and_rotation(
                thumb_base=_chain_transforms[self.finger_to_base["thumb"]]
                .transform_points(self.root)
                .cpu()
                .numpy(),
                index_base=_chain_transforms[self.finger_to_base["index"]]
                .transform_points(self.root)
                .cpu()
                .numpy(),
                middle_base=_chain_transforms[self.finger_to_base["middle"]]
                .transform_points(self.root)
                .cpu()
                .numpy(),
                ring_base=_chain_transforms[self.finger_to_base["ring"]]
                .transform_points(self.root)
                .cpu()
                .numpy(),
                pinky_base=_chain_transforms[self.finger_to_base["pinky"]]
                .transform_points(self.root)
                .cpu()
                .numpy(),
                wrist=_chain_transforms[WRIST_NAME]
                .transform_points(self.root)
                .cpu()
                .numpy(),
            )
        )

        assert np.allclose(
            (self.model_rotation @ self.model_rotation.T), (np.eye(3)), atol=1e-6
        ), "Model rotation matrix is not orthogonal"

    def sanity_check(self):
        """
        Check if the chain and scheme configuration is correct
        """

        ## Check the tip and base frames exist
        for finger, tip in self.finger_to_tip.items():
            assert (
                tip in self.chain.get_link_names()
            ), f"Tip frame {tip} not found in the chain"
        for finger, base in self.finger_to_base.items():
            assert (
                base in self.chain.get_link_names()
            ), f"Base frame {base} not found in the chain"
        for finger, mp in self.finger_to_mp.items():
            assert (
                mp in self.chain.get_link_names()
            ), f"MP frame {mp} not found in the chain"

        ## Check the base frame is fixed to the palm
        chain_transform1 = self.chain.forward_kinematics(
            torch.randn(self.chain.n_joints, device=self.chain.device)
        )
        chain_transform2 = self.chain.forward_kinematics(
            torch.randn(self.chain.n_joints, device=self.chain.device)
        )
        chain_transform3 = self.chain.forward_kinematics(
            torch.randn(self.chain.n_joints, device=self.chain.device)
        )
        for finger, base in self.finger_to_base.items():
            print(
                chain_transform1[base].transform_points(self.root),
                chain_transform1[base].transform_points(self.root),
                chain_transform1[base].transform_points(self.root),
            )
            assert torch.allclose(
                chain_transform1[base].transform_points(self.root),
                chain_transform2[base].transform_points(self.root),
            ), f"Base frame {base} not fixed to the palm"
            assert torch.allclose(
                chain_transform1[base].transform_points(self.root),
                chain_transform2[base].transform_points(self.root),
            ), f"Base frame {base} not fixed to the palm"

    def retarget_finger_mano_joints(
        self,
        joints,
        warm: bool = True
    ):
        """
        Process the MANO joints and update the finger joint angles
        joints: (21, 3)
        Over the 21 dims:
        0-4: thumb (from hand base)
        5-8: index
        9-12: middle
        13-16: ring
        17-20: pinky
        """

        opt_steps = RETARGETER_PARAMS["opt_steps"]

        print(f"Retargeting: Warm: {warm} Opt steps: {opt_steps}")

        start_time = time.time()
        if not warm:
            self.gc_joints = torch.ones(self.n_joints).to(self.device) * float(self.motor_count)
            self.gc_joints.requires_grad_()

        assert joints.shape == (
            21,
            3,
        ), "The shape of the mano joints array should be (21, 3)"

        joints = torch.from_numpy(joints).to(self.device)

        mano_joints_dict = retarget_utils.get_mano_joints_dict(joints)

        mano_fingertips = {}
        for finger, finger_joints in mano_joints_dict.items():
            mano_fingertips[finger] = finger_joints[[-1], :]

        mano_mps = {}
        for finger, finger_joints in mano_joints_dict.items():
            mano_mps[finger] = finger_joints[[1], :]

        mano_pps = {}
        for finger, finger_joints in mano_joints_dict.items():
            mano_pps[finger] = finger_joints[[0], :]

        mano_palm = torch.mean(
            torch.cat([mano_pps["thumb"], mano_pps["pinky"]], dim=0).to(self.device),
            dim=0,
            keepdim=True,
        )

        keyvectors_mano = retarget_utils.get_keyvectors(mano_fingertips, mano_mps, mano_palm)
        # norms_mano = {k: torch.norm(v) for k, v in keyvectors_mano.items()}
        # print(f"keyvectors_mano: {norms_mano}")

        for _ in range(opt_steps):
            chain_transforms = self.chain.forward_kinematics(
                self.joint_map @ (self.gc_joints / (180 / np.pi))
            )
            fingertips = {}
            for finger, finger_tip in self.finger_to_tip.items():
                fingertips[finger] = chain_transforms[finger_tip].transform_points(
                    self.root
                )

            mps = {}
            for finger, finger_mp in self.finger_to_mp.items():
                mps[finger] = chain_transforms[finger_mp].transform_points(self.root)

            palm = chain_transforms["retarget_palm"].transform_points(
                    self.root
                )

            keyvectors_faive = retarget_utils.get_keyvectors(fingertips, mps, palm)
            # norms_faive = {k: torch.norm(v) for k, v in keyvectors_faive.items()}
            # print(f"keyvectors_faive: {norms_faive}")

            loss = 0

            for i, (keyvector_faive, keyvector_mano) in enumerate(
                zip(keyvectors_faive.values(), keyvectors_mano.values())
            ):
                keyvector_faive *= self.scale_coeffs[i]
                if not self.use_scalar_distance[i]:
                    loss += (
                        self.loss_coeffs[i]
                        * torch.norm(keyvector_mano - keyvector_faive) ** 2
                    )
                else:
                    loss += (
                        self.loss_coeffs[i]
                        * (torch.norm(keyvector_mano) - torch.norm(keyvector_faive))
                        ** 2
                    )

            # Regularize the joints to zero
            loss += torch.sum(
                self.regularizer_weights * (self.gc_joints - self.regularizer_zeros) ** 2
            )

            # print(f"step: {step} Loss: {loss}")
            self.scaling_factors_set = True

            self.opt.zero_grad()

            loss.backward() # type: ignore
            self.opt.step()
            with torch.no_grad():
                self.gc_joints[:] = torch.clamp(
                    self.gc_joints,
                    torch.tensor(self.gc_limits_lower).to(self.device),
                    torch.tensor(self.gc_limits_upper).to(self.device),
                )

        finger_joint_angles = self.gc_joints.detach().cpu().numpy()

        print(f"Retarget time: {(time.time() - start_time) * 1000} ms")

        return finger_joint_angles, mano_fingertips, mano_mps, mano_palm, fingertips, mps, palm


    def adjust_mano_fingers(self, joints):


        # Assuming mano_adjustments is accessible within the class
        mano_adjustments = self.mano_adjustments

        # Get the joints per finger
        joints_dict = retarget_utils.get_mano_joints_dict(
            joints, include_wrist=True, batch_processing=False
        )

        # Initialize adjusted joints dictionary
        adjusted_joints_dict = {}

        # Process each finger
        for finger in ["thumb", "index", "middle", "ring", "pinky"]:
            # Original joints for the finger
            finger_joints = joints_dict[finger]  # Shape: (n_joints, 3)

            if  mano_adjustments.get(finger) is None:
                adjusted_joints_dict[finger] = finger_joints
                continue
            # Adjustments for the finger
            adjustments = mano_adjustments[finger]
            translation = adjustments.get("translation", np.zeros(3))  # (3,)
            rotation_angles = adjustments.get("rotation", np.zeros(3))  # (3,)
            scale = adjustments.get("scale", np.ones(3))  # (3,)

            # Scaling in the finger base frame
            x_base = finger_joints[0]  # Base joint position (3,)
            x_local = finger_joints - x_base  # Local coordinates (n_joints, 3)
            x_local_scaled = x_local * scale  # Apply scaling

            # Rotation around base joint in palm frame
            rot = Rotation.from_euler("xyz", rotation_angles, degrees=False)
            R_matrix = rot.as_matrix()  # Rotation matrix (3,3)
            x_local_rotated = x_local_scaled @ R_matrix.T  # Apply rotation
            finger_joints_rotated = x_base + x_local_rotated  # Rotated positions

            # Translation in palm frame
            finger_joints_adjusted = finger_joints_rotated + translation  # Adjusted positions

            # Store adjusted joints
            adjusted_joints_dict[finger] = finger_joints_adjusted

        # Keep the wrist as is
        adjusted_joints_dict["wrist"] = joints_dict["wrist"]

        # Concatenate adjusted joints
        joints = np.concatenate(
            [
                adjusted_joints_dict["wrist"].reshape(1, -1),
                adjusted_joints_dict["thumb"],
                adjusted_joints_dict["index"],
                adjusted_joints_dict["middle"],
                adjusted_joints_dict["ring"],
                adjusted_joints_dict["pinky"],
            ],
            axis=0,
        )

        return joints


    def retarget(self, joints, debug_dict):
        normalized_joint_pos, mano_center_and_rot = (
            retarget_utils.normalize_points_to_hands_local(joints)
        )
        normalized_joint_pos = self.adjust_mano_fingers(normalized_joint_pos)
        # (model_joint_pos - model_center) @ model_rotation = normalized_joint_pos
        debug_dict["mano_center_and_rot"] = mano_center_and_rot
        debug_dict["model_center_and_rot"] = (self.model_center, self.model_rotation)
        normalized_joint_pos = (
            normalized_joint_pos @ self.model_rotation.T + self.model_center
        )
        if debug_dict is not None:

            shifted_points = normalized_joint_pos + RETARGETER_PARAMS["mano_shift"]
            debug_dict["normalized_joint_pos"] = shifted_points
            debug_dict["original_joint_pos"] = joints

        self.target_angles, mano_fingertips, mano_mps, mano_palm, fingertips, mps, palm = self.retarget_finger_mano_joints(
            normalized_joint_pos
        )

        return self.target_angles, mano_fingertips, mano_mps, mano_palm, fingertips, mps, palm
