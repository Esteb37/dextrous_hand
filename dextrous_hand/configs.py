
#!/usr/bin/env python3

from dextrous_hand.HandConfig import HandConfig

HOME = HandConfig.default()

GRASP = HandConfig(
	PINKY=[2.288, 2.253, 1.958],
	RING=[1.834, 1.868, 2.075],
	MIDDLE=[2.037, 2.013, 1.975],
	INDEX=[2.034, 2.263, 2.029],
	THUMB=[0.000, 0.000, 0.000],
	WRIST=[0.000],
)

ROCK = HandConfig(
	PINKY=[0.000, 0.083, -0.592],
	RING=[1.835, 1.867, 2.072],
	MIDDLE=[1.783, 2.002, 2.076],
	INDEX=[0.001, -0.000, 0.001],
	THUMB=[0.000, 0.000, 0.000],
	WRIST=[0.000],
)


FINGER = HandConfig(
	PINKY=[2.288, 2.322, 1.958],
	RING=[1.834, 1.868, 2.075],
	MIDDLE=[0.029, -0.079, 0.336],
	INDEX=[2.034, 2.263, 2.027],
	THUMB=[0.000, 0.000, 0.000],
	WRIST=[0.000],
)
