
#!/usr/bin/env python3

from dextrous_hand.HandConfig import HandConfig

HOME = HandConfig.default()

GRASP = HandConfig(
	PINKY=[0.000, 2.265, 2.030],
	RING=[0.000, 1.728, 2.115],
	MIDDLE=[0.000, 2.107, 1.996],
	INDEX=[0.000, 2.189, 2.063],
	THUMB=[0.000, 0.000, 0.000],
	WRIST=[0.000],
)


ROCK = HandConfig(
	PINKY=[0.000, 0.000, 0.000],
	RING=[0.000, 1.728, 2.115],
	MIDDLE=[0.000, 2.107, 1.996],
	INDEX=[0.000, 0.000, 0.000],
	THUMB=[0.000, 0.000, 0.000],
	WRIST=[0.000],
)

FINGER = HandConfig(
	PINKY=[0.000, 2.265, 2.030],
	RING=[0.000, 1.728, 2.115],
	MIDDLE=[0.000, 0.000, 0.000],
	INDEX=[0.000, 2.189, 2.063],
	THUMB=[0.000, 0.000, 0.000],
	WRIST=[0.000],
)


LOVE = HandConfig(
	PINKY=[0.000, 2.265, 2.030],
	RING=[0.000, 1.728, 2.115],
	MIDDLE=[0.000, 0.000, 0.000],
	INDEX=[0.000, 0.000, 0.000],
	THUMB=[0.000, 0.000, 0.000],
	WRIST=[0.000],
)
