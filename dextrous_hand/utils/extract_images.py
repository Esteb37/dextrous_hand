#!/bin/env python3

import os
import glob
import argparse
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d
import numpy as np
import h5py
import cv2
from dextrous_hand.utils.constants import IMAGE_TOPIC_TYPES  # Import the predefined topic types
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import subprocess

TOPIC_TO_STRING = {
    Float32MultiArray: "Float32MultiArray",
    PoseStamped: "PoseStamped",
    Image: "Image",
    String: "String",
}

def get_topic_names(h5_path):
    with h5py.File(h5_path, 'r') as h5_file:
        topic_names = list(h5_file.keys())
        print(f"Topics in the HDF5 file: {topic_names}")
    return topic_names

def sample_and_sync_h5(input_h5_path, output_h5_path, sampling_frequency, compress, resize_to, topic_types):
    """
    Sample images and interpolate data for synchronization.

    Parameters:
        input_h5_path (str): Path to the input HDF5 file.
        output_h5_path (str): Path to the output HDF5 file.
        sampling_frequency (float): Sampling frequency in Hz.
        topic_types (dict): Dictionary mapping topics to their types.
    """

    with h5py.File(input_h5_path, 'r') as input_h5:
        # Determine sampling timestamps
        # Process each topic
        for topic, topic_type in topic_types.items():
            if topic not in input_h5:
                print(f"Topic {topic} not found in the HDF5 file. Skipping...")
                continue


            print(f"Processing topic: {topic}")
            topic_group = input_h5[topic]

            image = topic_group[0][:] # type: ignore

            # RGB to BGR
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            file_name = f"{topic}_{input_h5_path}.jpg".replace("/", "_")
            print(file_name)

            # Save the image
            cv2.imwrite(f"/media/esteb37/Data/images/{file_name}", image)


def get_git_commit_hash(input_folder):

    try:
        current_dir = os.getcwd()
        # Find the top-level directory of the Git repository
        os.chdir(input_folder)

        top_level_dir = subprocess.check_output(['git', 'rev-parse', '--show-toplevel']).strip().decode('utf-8')
        print(f"Top-level directory: {top_level_dir}")
        # Change the working directory to the top-level directory
        os.chdir(top_level_dir)

        # Get the commit hash
        commit_hash = subprocess.check_output(['git', 'rev-parse', 'HEAD']).strip().decode('utf-8')
        os.chdir(current_dir)
        return commit_hash[:7]  # Use the first 7 characters of the commit hash
    except subprocess.CalledProcessError:
        return "unknown"

def process_folder(input_folder, sampling_frequency, compress, resize_to, topic_types):
    """
    Process all HDF5 files in the given folder and save the processed files
    with a running index in a new folder named <input_folder>_processed.

    Parameters:
        input_folder (str): Path to the folder containing input HDF5 files.
        sampling_frequency (float): Sampling frequency in Hz.
        topic_types (dict): Dictionary mapping topics to their types.
    """
    # Get all HDF5 files in the folder
    h5_files = sorted(glob.glob(os.path.join(input_folder, "*.h5")))
    if not h5_files:
        print(f"No HDF5 files found in {input_folder}.")
        return

    # Create the output folder
    commit_hash = get_git_commit_hash(input_folder)
    output_folder = os.path.join(os.path.dirname(input_folder),
                                 os.path.basename(input_folder) + f"_processed_{commit_hash}_{int(sampling_frequency)}hz")
    if compress:
        output_folder += "_lzf"


    # Process each file
    for idx, input_file in enumerate(h5_files):
        try:
            # output_file = os.path.join(output_folder, f"{idx:04d}.h5")
            print(f"Processing file: {input_file}")
            sample_and_sync_h5(input_file, output_folder, sampling_frequency, compress, resize_to, topic_types)
        except Exception as e:
            print(e)

    print(f"All files processed. Processed files are saved in {output_folder}.")

def main():
    parser = argparse.ArgumentParser(description="Process and synchronize HDF5 files.")
    parser.add_argument("input_folder", type=str, help="Path to the folder containing input HDF5 files.")
    parser.add_argument("--sampling_freq", type=float, default=50, help="Sampling frequency in Hz.")
    parser.add_argument("--compress",  action="store_true", help="Compress the output HDF5 files. [it might boost the performance on aws but might decrease the performance on local machine]")
    parser.add_argument(
        '--resize_to',
        type=lambda s: tuple(map(int, s.strip("()").split(","))),
        help="Target size of the image as a tuple of integers, e.g., '(width, height)'.",
        default=None
    )
    args = parser.parse_args()

    # Process all files in the folder
    process_folder(args.input_folder, args.sampling_freq, args.compress, args.resize_to, IMAGE_TOPIC_TYPES)

if __name__ == "__main__":
    main()
