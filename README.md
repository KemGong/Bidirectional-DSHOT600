# Bidirectional-DSHOT600
Bidirectional DSHOT

历史

在Dshot之前，有许多模拟协议，其中最广为人知的便是PWM，还有其他的协议，例如Oneshot和Multishot。在四轴爱好者中最受欢迎的模拟协议是Multishot协议。

与模拟协议相比，数字协议具有许多好处：
错误检查：校验和检查允许电调确认信号确实是由飞控发送并在传输过程中没有受到干扰（一定程度上）

更高的分辨率：Dshot下的油门分辨率为2000

没有晶振漂移，因此无需校准

可在一根电线上完成双向通信


但是，事物均有两面性，数字协议也是如此。缺点是，数字协议并不是最快的，因为他们带来了一定的开销，例如Dshot中的CRC，这虽然增加了可靠性，但也延长了传输数据与信号帧的长度。此外，Dshot的信号帧的长度是固定的，不论是全油门还是零油门——与之相对的是，在模拟协议下，当油门值较小时，模拟脉冲的长度则较长。

Multishot在全油门时的帧最大持续时间约为25us，它比Dshot300的固定帧长53.28us快两倍还多。

支持的硬件

所有BLHeli_S、BLHeli_32和KISS电调都支持Dshot协议。这里的一个限制因素是一些使用EFM8BB1单片机的较旧的BLHeli_S电调：他们只支持Dshot150和Dshot 300，但这对于99%的情况来说仍然够用。‘

不需要对电调进行额外的设置，他们会自动检测驱动他们的协议并相应地切换到这个写一下。不过，某些固件可能仅支持特定的协议，因此请格外注意这一点，例如Bluejay仅支持Dshot协议，而并不支持任何模拟协议。

帧结构

每个数字协议都有一个结构，也称为帧。它定义了哪些信息位于数据流中的哪个位置。Dshot的帧结构其实非常简单：

11位油门信号：共有2048个不同的值。0保留为上锁命令，1-47被保留为特殊命令。剩下的48-2047（共2000步）用于实际的油门值

1位回传请求：如果设置了这一位，那么遥测数据将通过另外一根专线（电调回传线）单独发回飞控

4位CRC(Cyclic Redundancy Checksum，循环冗余校验)：检验数据的有效性（包括油门数据以及回传请求位）


生成具有以下结构的16位（2字节）帧：
SSSSSSSSSSSTCCCC

有趣的是，Dshot帧中的1和0是以它们不同的高电平时间来区分的。这意味每个位都有恒定的长度，并且以该位中高电平的持续时长来确定接收到的信号到底是0还是1。
图片


这有两个好处：
每一帧都具有完全相同的、易于计算的持续时长：16 x 位的时长

位的测量总是可以在上升沿触发，并在下降沿截止（如果是双向dshot的反向信号则相反，下降沿触发，上升沿截止）


在Dshot中，1的高电平时长是0的两倍。帧的实际持续时长、位的周期时长和帧长度取决于Dshot版本：
Dshot
比特率
T1H
T0H
位(μs)
帧(μs)
150
150kbit/s
5.00
2.50
6.67
106.72
300
300kbit/s
2.50
1.25
3.33
53.28
600
600kbit/s
1.25
0.625
1.67
26.72
1200
1200kbit/s
0.625
0.313
0.83
13.28

T1H表示的是逻辑为1的信号位持续时间，单位为微秒；T0H表示的是逻辑为0的信号位持续时间。

特殊命令

如上一节所述，油门值得0-47均是为特殊命令所保留的：
图片



命令0-36仅可以在电机停止时执行。一些命令需要多次发送——那些Need nx标记——其中n是命令必须发送的次数，以便于电调能够响应这些操作。

为了避免复杂，下面是BLHeli_32电调的ESC_INFO响应帧：
图片


计算CRC

校验和是根据油门数据和遥测位来计算的，因此，下面中的value表示的是前12位数据。
crc = (value ^ (value >> 4) ^ (value >>8)) & 0x0f;

假设我们正在发送一个1046的油门值（正好是全油门的一半），并且遥测位为0：
value  = 100000101100
(>>4)  = 000010000010 # right shift value by 4
(^)    = 100010101110 # XOR with value
(>>8)  = 000000001000 # right shift value by 8
(^)    = 100010100110 # XOR with previous XOR
(0x0F) = 000000001111 # Mask 0x0F
(&)    = 000000000110 # CRC
所以，从飞控传输到电调的两个字节的数据是：
1000001011000110

我们会将这个Dshot数据帧放在电线上以传输：
图片


绿色部分是油门位，蓝色是遥测位，黄色是CRC校验和。


为什么帧长这么重要？

帧的长度很重要，因为它表示的是电调的更新速率。帧的长度越短，那么每秒发送数据帧的频率就越高。换句话说，比特率越高，我们每秒钟可以发送的数据就越多。

它仅仅受到飞控的控制环路速率限制。或者，反过来看：

我们来看一下Dshot300：106.72μs的帧长，理论上允许我们每秒发送18768个完整的帧，也就是说它的最大频率约为18kHz。

从这个例子中我们可以得出结论：当PID环路频率为8kHz时，我们无法跑满Dshot300，因此，当您的PID频率为8kHz或更低时，没有任何实际理由非要运行Dshot600。

但事实并不完全如此，因为飞控会将帧与帧间隔开，并将其锁定到PID频率。因此Dshot始终以PID频率运行。另一方面，这意味着，如果您运行的PID频率非常高，则还需要运行更快的Dshot。

例如，如果您的PID频率为32kHz，飞控将每隔31.25μs发送一次Dshot帧，这意味着您必须至少运行Dshot600。


什么是电调遥测？

在关于帧的部分中，我提到了遥测位。飞控使用该位数据像电调请求遥测信息。

遥测信息可以是各种不同的东西，例如电调的温度、电机的eRPM、电流和电压等。

注意，电调遥测不是双向Dshot，并且由于其通信速度太慢，这导致RPM滤波无法正常工作。

硬件兼容性

电调遥测需要使用一根单独的电线将遥测信息回传给飞行控制器。通常来说仅有BLHeli_32和KISS电调支持这一功能。多个电调可以共享同一根连回传线，并连接到飞控上的UART的TX（或RX）引脚上。

都有哪些遥测数据？

如上一节所述，油门值1-47被保留用于特殊命令，其中一些用于请求遥测数据。在这些命令中，42-47与遥测相关，请参考表格以查看您可以查询那些遥测数据。

传输

当遥测位为1时，所请求的信息可以通过专用的回传线，按照KISS电调遥测协议将数据返回飞控发送给飞控。多个电调可以共享同一根回传线。

遥测数据帧长高达10字节（80位），以115200的波特率进行传输。

所有遥测数据都在此帧中进行传输。在这里我并不想进一步详细介绍电调遥测协议，因为实际上这并不是Dshot的一部分。详细的规范可以在RCGroups的线程中找到。

这种查询方式已经过时了，而且速度太慢，除非您对电调上的电流消耗感兴趣，基本上它无法做任何有意义的事情。


双向Dshot

您可以在BLHeli_32，AM32和其他一些第三方的BLHeli_S固件上使用双向Dshot，关于其余第三方BLS电调固件，可以参阅先前推送的文章。

双向Dshot也被称为反相Dshot，因为它的信号电平是反相的，所以低电平为1，高电平为0。这样做是为了让电调知道我们正在双向模式下进行通信，并且电调应该发挥ePRM回传包。

双向Dshot仅适用于Dshot300和Dshot600

计算校验和

校验和的计算步骤基本相同，只是在进行最后一步之前进行取反操作：
crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;

使用与常规Dshot帧相同的值：
value  = 100000101100
(>>4)  = 000010000010 # right shift value by 4
(^)    = 100010101110 # XOR with value
(>>8)  = 000000001000 # right shift value by 8
(^)    = 100010100110 # XOR with previous XOR
(~)    = 011101011001 # Invert
(0x0F) = 000000001111 # Mask 0x0F
(&)    = 000000001001 # CRC

双向Dshot帧（发送自飞控）

飞控发送到电调的帧结构完全一样，只是信号取反。从飞控传输到电调的两个字节数据将是：
1000001011001001

在电线上，信号看起来是这个样子：
图片


启用双向Dshot之后，对于每个发送到电调的Dshot帧，都会返回一个带有eRPM回传数据的帧（在同一条线上，而不是在另外一根遥测线上），这使得每秒可以发送的有效帧的数量减半。您需要牢记这一点，尤其是在您运行更高的PID频率时。

尽管在双向Dshot模式下，一个Dshot帧会始终返回一个eRPM回传帧，但您仍可以请求其他遥测信息，不过随后会通过单独的遥测线发送回飞控的UART。

一旦飞控发送了Dshot帧，它就会切换到接受模式并等待接受从电调返回的eRPM帧。同样的事情也发生在电调上，当飞控发送Dshot数据时，电调处于接受模式，反之亦然。

eRPM回传帧（发送自电调）

电调在双向Dshot模式下发送的eRPM又是一个16位的值，所以这和电调接收到的帧的大小相同，但结构不同。
12位：eRPM数据

4位：CRC


eRPM数据的编码并不像油门数据那样直接：
3位：需要将接下来九位数据左移的位数，以便于将周期数据的单位转换成μs

9位：待操作的周期数据

eeemmmmmmmmmcccc

CRC的计算与未反相的Dshot完全相同，它也将被原封不动地发回给飞控。

eRPM传输

但这里出现了一个转折。实际上，并不是将这个值直接发回给飞控，而是使用GCR编码，并根据下表，将16位值通过半字节（4位）映射到20位值：
原值    0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F  
映射值  19  1B  12  13  1D  15  16  17  1A  09  0A  oB  1E  oD  0E  0F


例如，如果我们使用下面这个数字：
16 bit:  1000001011000110
Nibbles:  1000  0010  1100  0110
Hex:        x8    x2    xC    x6
Mapped:    x1A   x12   x1E   x16
GCR:   11000 10010 11100 10110
GCR:   11000100101110010110

现在我们将16位值映射到20位，但这并没有准备好进行传输，还需要添加一个第21位，并按照下面规则转换原始数据：

我们将GCR编码后的数据映射成一个21位的新数据，这个新值以0开头，其余的位则遵循下面两个规则生成：
如果GCR数据的此位数据为1：当前的新位是最后一个新位的反转；

如果GCR数据的此位数据为0：当前的新位与最后一个新位相同。


用一个简短的例子来解释它，假设我们的GCR数据为01100010：
GCR:  01100010
New: 0          # We start out with adding a 0 bit (the 25th bit in the real frame)
     00         # 1 bit of GCR is 0 => Rule 1: new bit is 0, because the last was 0
     001        # 2 bit of GCR is 1 => Rule 2: new bit is 1 after inverting the last bit
     0010       # 3 bit of GCR is 1 => Rule 2: new bit is 0 after inverting the last bit
     00100      # 4 bit of GCR is 0 => Rule 2: new bit is 0, because the last was 0
     001000     # 5 bit of GCR is 0 => Rule 2: new bit is 0, because the last was 0
     0010000    # 6 bit of GCR is 0 => Rule 2: new bit is 0, because the last was 0
     00100001   # 7 bit of GCR is 1 => Rule 2: new bit is 1 after inverting the last bit
     001000011  # 8 bit of GCR is 0 => Rule 2: new bit is 1, because the last was 1

让我们看一个真实的20位GCR数据：最坏的情况，邻位都相互不同
GCR:  10101010101010101010
New: 011001100110011001100

当我们把这个值放在电线上时，高低电平之间的电平反转次数仅有原始GCR数据的一半。

所以，我们并没有发送这个：
图片


而是发送了这个：
图片


然后以5/4 x 当前Dshot比特率的比特率原封不动地发送该值。在Dshot600上，21位数据将以750kbit/s的比特率进行发送。

现在，您或许会问自己：为什么要这么麻烦？事实证明，GCR是一种非常出色的传输模式，对于干扰与抖动十分健壮。实测表明，与将eRPM回传帧不经任何编码直接发回给飞控相比，使用GCR编码后的eRPM数据包的错误率要低得多。

解码eRPM帧（在飞控上）

在接收端，解码eRPM帧也很简单：
gcr = (value ^ (value >> 1));
该值只需要在右移一次之后与自身进行异或运算即可：
value = 011001100110011001100
(>>1) = 001100110011001100110 # right shift value by 1
(^)   = 010101010101010101010 # GCR