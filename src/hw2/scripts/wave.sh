rostopic pub -1 trajectory_multi_command hw2/TrajectoryMultiCommand -- '{points: [
{time: 0, t_k: {data: {secs: 5, nsecs: 0}}, t_k_prime: {data: {secs: 1, nsecs: 0}}, names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7'], q_final: [0.787,0.787,0.0,0.0,0.0,0.0,0.0], qdot_final: [0,0,0,0,0,0,0]},
{time: 0, t_k: {data: {secs: 5, nsecs: 0}}, t_k_prime: {data: {secs: 1, nsecs: 0}}, names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7'], q_final: [-0.787,-0.787,0.0,0.0,0.0,0.0,0.0], qdot_final: [0,0,0,0,0,0,0]},
{time: 0, t_k: {data: {secs: 5, nsecs: 0}}, t_k_prime: {data: {secs: 1, nsecs: 0}}, names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7'], q_final: [0.0,0.0,0.0,0.0,0.0,0.0,0.0], qdot_final: [0,0,0,0,0,0,0]}]}'
