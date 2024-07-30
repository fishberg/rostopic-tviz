# rostopic-tviz

Simple tools for visualizing rostopics in the terminal. Scripts do NOT use command-line arguments and are expected to be directly edited for each experimental setup. (This is so you don't need to type a long list of command-line arguments during an experiment.)

## Quick Launch

### mocap_tviz.py

```
# terminal 1
$ roscore

# terminal 2
$ rosbag play -l bag/mocap.bag

# terminal 3
$ ./mocap_tviz.py
+--------+------+-------+------+--------+---------+---------+
|        |    x |     y |    z |   roll |   pitch |     yaw |
|--------+------+-------+------+--------+---------+---------|
| agent1 | 0.90 | -3.15 | 0.86 |   0.76 |    0.25 |  111.02 |
| agent2 | 1.41 | -3.27 | 0.85 |   1.20 |   -0.51 | -138.44 |
+--------+------+-------+------+--------+---------+---------+
```

### imu_tviz.py

```
# terminal 1
$ roscore

# terminal 2
$ rosbag play -l bag/vectornav.bag

# terminal 3
$ ./mocap_tviz.py
+-------------+---------+---------+---------+------------+-------------+-----------+--------+---------+--------+
|             |   acc_x |   acc_y |   acc_z |   vel_roll |   vel_pitch |   vel_yaw |   roll |   pitch |    yaw |
|-------------+---------+---------+---------+------------+-------------+-----------+--------+---------+--------|
| microstrain |  nan    |  nan    |  nan    |     nan    |      nan    |    nan    | nan    |  nan    | nan    |
| vectornav   |    7.10 |   -0.26 |   -6.83 |       0.11 |       -0.13 |      0.07 |   1.70 |   46.01 | -12.99 |
+-------------+---------+---------+---------+------------+-------------+-----------+--------+---------+--------+

```

```
# terminal 1
$ roscore

# terminal 2
$ rosbag play -l bag/microstrain.bag

# terminal 3
$ ./mocap_tviz.py
+-------------+---------+---------+---------+------------+-------------+-----------+--------+---------+--------+
|             |   acc_x |   acc_y |   acc_z |   vel_roll |   vel_pitch |   vel_yaw |   roll |   pitch |    yaw |
|-------------+---------+---------+---------+------------+-------------+-----------+--------+---------+--------|
| microstrain |   -0.95 |    0.40 |    9.95 |       0.15 |        0.19 |      0.49 |  -4.71 |    0.79 | 176.43 |
| vectornav   |  nan    |  nan    |  nan    |     nan    |      nan    |    nan    | nan    |  nan    | nan    |
+-------------+---------+---------+---------+------------+-------------+-----------+--------+---------+--------+
```

### uwb_tviz.py

```
# SHOW_ONLY_SPECIFIED
# :: True: cols will only be shown for IDs specified in A_IDS + B_IDS
# :: False: cols will be shown for any ID heard from

# SHOW_SYMMETRIC
# :: True: shows A <=> B range measurements
# :: False: shows only A => B range measurments
```

```
# terminal 1
$ roscore

# terminal 2
$ rosbag play -l bag/uwb.bag

# terminal 3
# SHOW_ONLY_SPECIFIED = True
# SHOW_SYMMETRIC = True
$ ./mocap_tviz.py
+-----+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+
|     | n0               | n1               | n2               | n3               | n4               | n5               | n6               | n7               | n8               | n9               | n10              | n11              |
|-----+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------|
| n0  | n/a              | 0.11 ± 0.02 [25] | 0.36 ± 0.03 [25] | 0.45 ± 0.01 [24] | 0.32 ± 0.04 [25] | 0.13 ± 0.01 [25] | 3.28 ± 0.02 [25] | 2.99 ± 0.03 [25] | 3.68 ± 0.10 [25] | 2.92 ± 0.03 [25] | 3.12 ± 0.03 [25] | 3.33 ± 0.03 [25] |
| n1  | 0.11 ± 0.03 [25] | n/a              | 0.10 ± 0.03 [25] | 0.30 ± 0.06 [25] | 0.36 ± 0.02 [25] | 0.26 ± 0.04 [25] | 3.48 ± 0.03 [25] | 3.18 ± 0.03 [25] | 3.07 ± 0.03 [25] | 3.20 ± 0.04 [25] | 3.36 ± 0.03 [25] | 3.57 ± 0.02 [25] |
| n2  | 0.37 ± 0.03 [25] | 0.09 ± 0.05 [25] | n/a              | 0.10 ± 0.04 [25] | 0.34 ± 0.03 [25] | 0.44 ± 0.03 [25] | 3.84 ± 0.03 [25] | 3.50 ± 0.02 [25] | 3.38 ± 0.06 [25] | 3.36 ± 0.03 [25] | 3.49 ± 0.04 [25] | 3.94 ± 0.17 [25] |
| n3  | 0.45 ± 0.03 [26] | 0.30 ± 0.05 [26] | 0.10 ± 0.04 [26] | n/a              | 0.09 ± 0.06 [26] | 0.30 ± 0.05 [26] | 3.83 ± 0.05 [26] | 3.50 ± 0.05 [26] | 3.54 ± 0.04 [26] | 3.32 ± 0.19 [26] | 3.56 ± 0.04 [26] | 3.79 ± 0.03 [26] |
| n4  | 0.33 ± 0.03 [26] | 0.36 ± 0.03 [26] | 0.34 ± 0.04 [26] | 0.09 ± 0.04 [26] | n/a              | 0.09 ± 0.08 [26] | 3.57 ± 0.05 [26] | 3.34 ± 0.06 [26] | 3.41 ± 0.12 [26] | 3.06 ± 0.03 [26] | 3.31 ± 0.04 [26] | 4.14 ± 0.36 [26] |
| n5  | 0.13 ± 0.02 [26] | 0.25 ± 0.04 [26] | 0.44 ± 0.03 [26] | 0.31 ± 0.04 [26] | 0.09 ± 0.07 [26] | n/a              | 3.46 ± 0.06 [26] | 3.05 ± 0.04 [26] | 2.94 ± 0.05 [26] | 2.91 ± 0.03 [26] | 3.15 ± 0.03 [26] | 3.79 ± 0.04 [26] |
| n6  | 3.28 ± 0.02 [25] | 3.48 ± 0.03 [25] | 3.84 ± 0.03 [25] | 3.82 ± 0.04 [25] | 3.57 ± 0.03 [25] | 3.47 ± 0.04 [25] | n/a              | 0.05 ± 0.09 [25] | 0.35 ± 0.03 [25] | 0.41 ± 0.03 [25] | 0.38 ± 0.10 [25] | 0.10 ± 0.04 [25] |
| n7  | 2.99 ± 0.03 [25] | 3.18 ± 0.04 [25] | 3.50 ± 0.02 [25] | 3.50 ± 0.04 [25] | 3.34 ± 0.06 [25] | 3.06 ± 0.04 [25] | 0.05 ± 0.09 [25] | n/a              | 0.02 ± 0.05 [25] | 0.29 ± 0.03 [25] | 0.35 ± 0.03 [25] | 0.28 ± 0.04 [25] |
| n8  | 3.67 ± 0.10 [25] | 3.07 ± 0.04 [25] | 3.39 ± 0.06 [25] | 3.53 ± 0.05 [25] | 3.41 ± 0.19 [25] | 2.94 ± 0.05 [25] | 0.35 ± 0.05 [25] | 0.01 ± 0.04 [25] | n/a              | 0.19 ± 0.03 [25] | 0.31 ± 0.05 [25] | 0.47 ± 0.05 [25] |
| n9  | 2.92 ± 0.03 [25] | 3.20 ± 0.04 [25] | 3.35 ± 0.03 [25] | 3.32 ± 0.18 [25] | 3.06 ± 0.03 [25] | 2.91 ± 0.02 [25] | 0.42 ± 0.03 [25] | 0.28 ± 0.05 [25] | 0.19 ± 0.02 [25] | n/a              | 0.08 ± 0.07 [25] | 0.24 ± 0.05 [25] |
| n10 | 3.13 ± 0.04 [26] | 3.36 ± 0.03 [26] | 3.48 ± 0.04 [26] | 3.56 ± 0.06 [26] | 3.31 ± 0.03 [26] | 3.15 ± 0.03 [26] | 0.38 ± 0.08 [26] | 0.35 ± 0.03 [26] | 0.31 ± 0.06 [26] | 0.07 ± 0.10 [26] | n/a              | 0.15 ± 0.10 [26] |
| n11 | 3.32 ± 0.03 [25] | 3.57 ± 0.02 [25] | 4.01 ± 0.17 [25] | 3.79 ± 0.04 [25] | 4.17 ± 0.34 [25] | 3.78 ± 0.05 [25] | 0.10 ± 0.04 [25] | 0.28 ± 0.03 [25] | 0.47 ± 0.03 [25] | 0.24 ± 0.05 [25] | 0.14 ± 0.13 [25] | n/a              |
+-----+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+
```

```
# terminal 1
$ roscore

# terminal 2
$ rosbag play -l bag/uwb.bag

# terminal 3
# SHOW_ONLY_SPECIFIED = True
# SHOW_SYMMETRIC = False
$ ./mocap_tviz.py
+----+------------------+------------------+------------------+------------------+------------------+------------------+
|    | n6               | n7               | n8               | n9               | n10              | n11              |
|----+------------------+------------------+------------------+------------------+------------------+------------------|
| n0 | 3.29 ± 0.02 [25] | 2.99 ± 0.03 [25] | 3.68 ± 0.09 [25] | 2.92 ± 0.03 [25] | 3.12 ± 0.03 [25] | 3.32 ± 0.03 [25] |
| n1 | 3.48 ± 0.03 [25] | 3.18 ± 0.03 [25] | 3.07 ± 0.03 [25] | 3.20 ± 0.04 [25] | 3.35 ± 0.03 [25] | 3.57 ± 0.02 [25] |
| n2 | 3.84 ± 0.03 [25] | 3.50 ± 0.02 [25] | 3.38 ± 0.06 [25] | 3.36 ± 0.03 [25] | 3.49 ± 0.03 [25] | 3.93 ± 0.16 [25] |
| n3 | 3.83 ± 0.05 [25] | 3.50 ± 0.05 [25] | 3.54 ± 0.04 [25] | 3.32 ± 0.20 [25] | 3.56 ± 0.04 [25] | 3.79 ± 0.03 [25] |
| n4 | 3.58 ± 0.06 [25] | 3.34 ± 0.05 [25] | 3.41 ± 0.15 [25] | 3.07 ± 0.03 [25] | 3.31 ± 0.04 [25] | 4.14 ± 0.36 [25] |
| n5 | 3.47 ± 0.07 [25] | 3.05 ± 0.04 [25] | 2.94 ± 0.06 [25] | 2.91 ± 0.03 [25] | 3.15 ± 0.04 [25] | 3.78 ± 0.04 [25] |
+----+------------------+------------------+------------------+------------------+------------------+------------------+
```

```
# terminal 1
$ roscore

# terminal 2
$ rosbag play -l bag/uwb.bag

# terminal 3
# SHOW_ONLY_SPECIFIED = False
# SHOW_SYMMETRIC = False
$ ./mocap_tviz.py
+----+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+
|    | n0               | n1               | n2               | n3               | n4               | n5               | n6               | n7               | n8               | n9               | n10              | n11              | n12              | n13              | n14              | n15              | n16              | n17              |
|----+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------|
| n0 | n/a              | 0.11 ± 0.03 [25] | 0.36 ± 0.02 [25] | 0.45 ± 0.02 [25] | 0.32 ± 0.04 [25] | 0.13 ± 0.01 [25] | 3.28 ± 0.03 [25] | 2.99 ± 0.03 [25] | 3.68 ± 0.09 [25] | 2.93 ± 0.03 [25] | 3.12 ± 0.03 [25] | 3.33 ± 0.04 [25] | 3.79 ± 0.19 [25] | 3.52 ± 0.03 [25] | 3.91 ± 0.03 [25] | 4.41 ± 0.04 [25] | 9.86 ± 0.05 [25] | 3.56 ± 0.04 [25] |
| n1 | 0.11 ± 0.03 [25] | n/a              | 0.09 ± 0.05 [25] | 0.30 ± 0.05 [25] | 0.36 ± 0.03 [25] | 0.25 ± 0.05 [25] | 3.48 ± 0.03 [25] | 3.18 ± 0.03 [25] | 3.07 ± 0.03 [25] | 3.20 ± 0.04 [25] | 3.35 ± 0.03 [25] | 3.57 ± 0.01 [25] | 3.04 ± 0.02 [25] | 3.26 ± 0.03 [25] | 3.58 ± 0.04 [25] | 3.69 ± 0.05 [25] | 3.38 ± 0.01 [25] | 3.23 ± 0.04 [25] |
| n2 | 0.36 ± 0.03 [25] | 0.09 ± 0.05 [25] | n/a              | 0.09 ± 0.04 [25] | 0.34 ± 0.03 [25] | 0.43 ± 0.03 [25] | 3.83 ± 0.03 [25] | 3.51 ± 0.03 [25] | 3.38 ± 0.04 [25] | 3.36 ± 0.02 [25] | 3.49 ± 0.04 [25] | 4.02 ± 0.19 [25] | 3.92 ± 0.05 [25] | 2.97 ± 0.04 [25] | 3.25 ± 0.03 [25] | 3.31 ± 0.08 [25] | 3.38 ± 0.56 [25] | 2.92 ± 0.06 [25] |
| n3 | 0.45 ± 0.02 [24] | 0.30 ± 0.04 [24] | 0.09 ± 0.04 [24] | n/a              | 0.09 ± 0.08 [24] | 0.31 ± 0.03 [24] | 3.84 ± 0.05 [24] | 3.49 ± 0.06 [24] | 3.53 ± 0.03 [24] | 3.30 ± 0.02 [24] | 3.57 ± 0.03 [24] | 3.79 ± 0.02 [24] | 2.88 ± 0.43 [24] | 3.03 ± 0.04 [24] | 3.34 ± 0.05 [24] | 3.30 ± 0.04 [24] | 2.99 ± 0.06 [24] | 3.25 ± 0.17 [24] |
| n4 | 0.32 ± 0.03 [25] | 0.36 ± 0.03 [25] | 0.34 ± 0.03 [25] | 0.10 ± 0.04 [25] | n/a              | 0.11 ± 0.06 [25] | 3.58 ± 0.05 [25] | 3.34 ± 0.06 [25] | 3.43 ± 0.15 [25] | 3.07 ± 0.04 [25] | 3.30 ± 0.03 [25] | 4.22 ± 0.16 [25] | 3.07 ± 0.03 [25] | 3.31 ± 0.04 [25] | 3.55 ± 0.02 [25] | 3.49 ± 0.04 [25] | 3.26 ± 0.05 [25] | 3.07 ± 0.07 [25] |
| n5 | 0.13 ± 0.02 [25] | 0.25 ± 0.05 [25] | 0.43 ± 0.03 [25] | 0.31 ± 0.03 [25] | 0.07 ± 0.07 [25] | n/a              | 3.47 ± 0.05 [25] | 3.05 ± 0.03 [25] | 2.94 ± 0.07 [25] | 2.91 ± 0.03 [25] | 3.15 ± 0.04 [25] | 3.79 ± 0.06 [25] | 3.44 ± 0.04 [25] | 3.53 ± 0.04 [25] | 3.78 ± 0.08 [25] | 3.82 ± 0.05 [25] | 3.46 ± 0.04 [25] | 3.52 ± 0.25 [25] |
+----+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+
```

```
# terminal 1
$ roscore

# terminal 2
$ rosbag play -l bag/uwb.bag

# terminal 3
# SHOW_ONLY_SPECIFIED = False
# SHOW_SYMMETRIC = True
$ ./mocap_tviz.py
+-----+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+
|     | n0               | n1               | n2               | n3               | n4               | n5               | n6               | n7               | n8               | n9               | n10              | n11              | n12              | n13              | n14              | n15              | n16              | n17              |
|-----+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------|
| n0  | n/a              | 0.10 ± 0.02 [24] | 0.37 ± 0.02 [24] | 0.45 ± 0.02 [24] | 0.33 ± 0.03 [24] | 0.13 ± 0.02 [24] | 3.28 ± 0.03 [24] | 2.99 ± 0.03 [24] | 3.70 ± 0.08 [24] | 2.93 ± 0.02 [24] | 3.12 ± 0.03 [24] | 3.33 ± 0.03 [24] | 3.66 ± 0.44 [24] | 3.52 ± 0.03 [24] | 3.91 ± 0.04 [24] | 4.41 ± 0.05 [24] | 9.87 ± 0.05 [24] | 3.57 ± 0.03 [24] |
| n1  | 0.10 ± 0.03 [25] | n/a              | 0.10 ± 0.04 [25] | 0.30 ± 0.05 [25] | 0.36 ± 0.02 [25] | 0.25 ± 0.04 [25] | 3.48 ± 0.04 [25] | 3.18 ± 0.03 [25] | 3.08 ± 0.04 [25] | 3.21 ± 0.04 [25] | 3.36 ± 0.04 [25] | 3.57 ± 0.02 [25] | 3.04 ± 0.03 [25] | 3.25 ± 0.03 [25] | 3.58 ± 0.05 [25] | 3.68 ± 0.05 [25] | 3.38 ± 0.02 [25] | 3.22 ± 0.04 [25] |
| n2  | 0.37 ± 0.03 [25] | 0.10 ± 0.05 [25] | n/a              | 0.09 ± 0.04 [25] | 0.34 ± 0.03 [25] | 0.44 ± 0.03 [25] | 3.84 ± 0.02 [25] | 3.50 ± 0.03 [25] | 3.38 ± 0.06 [25] | 3.36 ± 0.03 [25] | 3.48 ± 0.04 [25] | 3.99 ± 0.18 [25] | 3.93 ± 0.04 [25] | 2.96 ± 0.04 [25] | 3.26 ± 0.04 [25] | 3.31 ± 0.08 [25] | 3.33 ± 0.62 [25] | 2.91 ± 0.06 [25] |
| n3  | 0.45 ± 0.03 [23] | 0.30 ± 0.04 [23] | 0.09 ± 0.05 [23] | n/a              | 0.09 ± 0.07 [23] | 0.31 ± 0.04 [23] | 3.82 ± 0.04 [23] | 3.49 ± 0.07 [23] | 3.53 ± 0.04 [23] | 3.29 ± 0.03 [23] | 3.56 ± 0.04 [23] | 3.79 ± 0.04 [23] | 2.80 ± 0.28 [23] | 3.04 ± 0.03 [22] | 3.33 ± 0.05 [23] | 3.30 ± 0.04 [23] | 2.99 ± 0.06 [23] | 3.26 ± 0.13 [23] |
| n4  | 0.33 ± 0.03 [25] | 0.36 ± 0.03 [25] | 0.34 ± 0.03 [25] | 0.10 ± 0.05 [25] | n/a              | 0.08 ± 0.08 [25] | 3.58 ± 0.06 [25] | 3.34 ± 0.07 [25] | 3.42 ± 0.14 [25] | 3.07 ± 0.03 [25] | 3.30 ± 0.03 [25] | 4.17 ± 0.31 [25] | 3.07 ± 0.04 [25] | 3.32 ± 0.06 [25] | 3.55 ± 0.02 [25] | 3.50 ± 0.03 [25] | 3.25 ± 0.04 [25] | 3.06 ± 0.05 [25] |
| n5  | 0.13 ± 0.02 [24] | 0.25 ± 0.04 [25] | 0.44 ± 0.03 [25] | 0.30 ± 0.04 [24] | 0.11 ± 0.06 [25] | n/a              | 3.46 ± 0.05 [25] | 3.05 ± 0.04 [25] | 2.94 ± 0.05 [25] | 2.91 ± 0.03 [25] | 3.15 ± 0.04 [25] | 3.79 ± 0.06 [25] | 3.44 ± 0.04 [25] | 3.53 ± 0.04 [25] | 3.80 ± 0.08 [25] | 3.81 ± 0.04 [25] | 3.45 ± 0.04 [25] | 3.54 ± 0.28 [24] |
| n6  | 3.29 ± 0.02 [25] | 3.48 ± 0.02 [25] | 3.84 ± 0.02 [25] | 3.82 ± 0.05 [25] | 3.58 ± 0.04 [25] | 3.47 ± 0.06 [25] | n/a              | 0.04 ± 0.08 [25] | 0.35 ± 0.03 [25] | 0.41 ± 0.02 [25] | 0.38 ± 0.09 [25] | 0.11 ± 0.03 [25] | 5.85 ± 0.03 [25] | 6.09 ± 0.04 [25] | 6.45 ± 0.04 [24] | 6.45 ± 0.04 [25] | 6.17 ± 0.04 [25] | 6.00 ± 0.03 [25] |
| n7  | 2.99 ± 0.04 [24] | 3.18 ± 0.03 [24] | 3.50 ± 0.03 [24] | 3.50 ± 0.05 [24] | 3.34 ± 0.07 [24] | 3.05 ± 0.04 [24] | 0.03 ± 0.08 [24] | n/a              | 0.01 ± 0.03 [24] | 0.29 ± 0.04 [24] | 0.35 ± 0.03 [24] | 0.28 ± 0.04 [24] | 5.53 ± 0.07 [24] | 5.80 ± 0.05 [24] | 6.08 ± 0.03 [24] | 6.22 ± 0.04 [24] | 5.93 ± 0.04 [24] | 5.63 ± 0.04 [24] |
| n8  | 3.70 ± 0.07 [25] | 3.08 ± 0.04 [25] | 3.39 ± 0.06 [25] | 3.53 ± 0.04 [25] | 3.41 ± 0.18 [25] | 2.94 ± 0.07 [25] | 0.35 ± 0.04 [25] | 0.00 ± 0.02 [25] | n/a              | 0.19 ± 0.02 [25] | 0.30 ± 0.05 [25] | 0.47 ± 0.04 [25] | 5.31 ± 0.03 [25] | 5.59 ± 0.03 [25] | 5.88 ± 0.03 [25] | 6.00 ± 0.03 [25] | 5.68 ± 0.04 [25] | 5.37 ± 0.03 [25] |
| n9  | 2.93 ± 0.03 [24] | 3.20 ± 0.04 [24] | 3.36 ± 0.04 [24] | 3.29 ± 0.03 [24] | 3.06 ± 0.03 [24] | 2.91 ± 0.02 [24] | 0.41 ± 0.03 [24] | 0.28 ± 0.05 [24] | 0.19 ± 0.02 [24] | n/a              | 0.07 ± 0.08 [24] | 0.24 ± 0.04 [23] | 5.41 ± 0.05 [24] | 5.74 ± 0.07 [24] | 6.05 ± 0.03 [24] | 5.89 ± 0.03 [24] | 5.62 ± 0.04 [24] | 5.39 ± 0.03 [24] |
| n10 | 3.12 ± 0.03 [25] | 3.37 ± 0.02 [25] | 3.49 ± 0.04 [25] | 3.56 ± 0.05 [25] | 3.31 ± 0.04 [25] | 3.15 ± 0.03 [25] | 0.38 ± 0.08 [25] | 0.35 ± 0.03 [25] | 0.30 ± 0.06 [25] | 0.06 ± 0.10 [25] | n/a              | 0.14 ± 0.11 [25] | 5.75 ± 0.07 [25] | 5.99 ± 0.05 [25] | 6.30 ± 0.05 [25] | 6.16 ± 0.04 [25] | 5.90 ± 0.03 [25] | 5.60 ± 0.05 [25] |
| n11 | 3.32 ± 0.03 [25] | 3.57 ± 0.03 [25] | 3.95 ± 0.24 [25] | 3.78 ± 0.05 [25] | 4.22 ± 0.17 [25] | 3.79 ± 0.07 [25] | 0.10 ± 0.04 [25] | 0.28 ± 0.04 [25] | 0.47 ± 0.05 [25] | 0.23 ± 0.05 [24] | 0.16 ± 0.15 [25] | n/a              | 6.06 ± 0.05 [25] | 6.24 ± 0.06 [25] | 6.40 ± 0.02 [25] | 6.51 ± 0.04 [25] | 6.19 ± 0.05 [25] | 5.96 ± 0.07 [25] |
+-----+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+------------------+
```