# 5737-2022-Program  
## 目前已知问题
自动阶段如何判断什么时候发球？  
自动阶段转进手动阶段时，自动程序还会自动执行。  
自动阶段期间，循环步骤会乱.  

## 20230711 update
## autonomousPeriodic-自动前进与自动旋转更改
```java
// 自动前进
public void go_straight(double meters,double enc){
    drive_left_1.set(auto_drivestraight(-enc, -meters));
    drive_right_2.set(auto_drivestraight(-enc, -meters));
}

public double auto_drivestraight(double current_encoder,double target_meter){
    double auto_straight_out = 0;
    double askp = 0.02;
    auto_straight_out = ((target_meter*21.65)+current_encoder)*askp;
    return auto_straight_out;
}
```
## shuffleboard-功能添加

```java
//创建新表格
ShuffleboardTab NewTab = Shuffleboard.getTab("NewTab");
private NetworkTableEntry GyroYaw = NewTab.add("GyroYaw",0).getEntry();
//更新表格 需要在不断调用的情况下进行更新
public void robotPeriodic() {
    GyroYaw.setDouble(gyro.getYaw());
}

```

## 20230712 update
## auto_detect 条件修改