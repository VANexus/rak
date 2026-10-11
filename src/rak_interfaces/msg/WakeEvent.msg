# R818 降噪板(讯飞 VTN)唤醒事件:板内 ivw 引擎检出唤醒词时上报一次。
# 同一事件自带声源方位——固件只在唤醒时报一次 angle,没有持续方位流
# (实测见 docs/microphone_array.md)。angle 是环形阵列自身坐标系的方位角,
# 不是机器人航向;要转头需再经标定/TF。

std_msgs/Header header  # stamp = 主机收到事件的时刻(板时钟不可信)

string keyword   # 板上注册的唤醒词拼音,如 "hei4 rui4 ke4"
float32 score    # 板固件标度的置信分;判定阈值存在板上 res 资源里,这里只是原始值
float32 angle    # 声源方位角(度,环形 0-365)
int32 beam       # 触发的波束编号(与 physical 同为板固件标度)
int32 physical   # 对应物理麦克风编号
float32 power    # 唤醒段能量(板固件标度,量纲未文档化)
int32 start_ms   # 唤醒段起止,毫秒(相对板启动,非主机时钟)
int32 end_ms
