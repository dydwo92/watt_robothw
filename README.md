# watt_robothw

## Configuration
config 폴더의 yaml 파일을 시스템에 맞게 수정

### config/system.yaml
```
control_rate : 50 # 컨트롤 주기. innfos_node와 zltech_node 모두 해당 값을 받아 적용
innfos_can_device : "vcan0" # innfos_node의 can device 이름
zltech_can_device : "vcan0" # zltech_node의 can device 이름
```

### config/innfos/hardware.yaml
```
DOF : 6 # 모터 갯수
joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'] # 각 모터 이름
joint_ids : [192, 193, 194, 195, 196, 197] # 각 모터 CAN ID
```

### config/zltech/hardware.yaml
```
DOF : 2 # 모터 갯수
joint_names: ['l_wheel_joint', 'r_wheel_joint'] # 각 모터 이름
joint_ids : [1, 2] # 각 모터의 CANOpen ID
```

그 외, config 내 파일 참고할 것

## Launching
launch 폴터에는 다음의 3가지 launch 파일이 있음
```
launch/innfos_hw.launch # innfos 모터만 구동
launch/zltech_hw.launch # zltech 모터만 구동
launch/watt_hw.launch # innfos 와 zltech 동시 
```
