### PCD文件格式
[PCD（点云数据）文件格式](http://www.pclcn.org/study/shownews.php?lang=cn&id=54)

PCL中PCD文件格式的正式发布是0.7版本（PCD_V7）。

#### 1、文件头格式
文件头格式示例:
```js
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 1
HEIGHT 214763
VIEWPOINT 0 0 0 1 0 0 0
POINTS 214763
DATA ascii
```

**1)VERSION**
指定PCD文件版本

**2)FIELDS**
指定一个点可以有的每一个维度和字段的名字。例如:
```js
FIELDS x y z                                   # XYZ data
FIELDS x y z rgb                               # XYZ + colors
FIELDS x y z normal_xnormal_y normal_z         # XYZ + surface normals
FIELDS j1 j2 j3                                # moment invariants
```
**3)SIZE**
用字节数指定每一个维度的大小。
```c
unsigned char/char has 1 byte
unsigned short/short has 2 bytes
unsignedint/int/float has 4 bytes
double has 8 bytes
```

**3)TYPE**
用一个字符指定每一个维度的类型。现在被接受的类型有：
```js
I –表示有符号类型int8（char）、int16（short）和int32（int）；
U – 表示无符号类型uint8（unsigned char）、uint16（unsigned short）和uint32（unsigned int）；
F –表示浮点类型。
```

**4)COUNT**
指定每一个维度包含的元素数目。

**5)WIDTH** 
用点的数量表示点云数据集的宽度。根据是有序点云还是无序点云，WIDTH有两层解释：
- 它能确定无序数据集的点云中点的个数（和下面的POINTS一样）；
- 它能确定有序点云数据集的宽度（一行中点的数目）。

**6)HEIGHT** 
用点的数目表示点云数据集的高度。类似于WIDTH ，HEIGHT也有两层解释：
- 它表示有序点云数据集的高度（行的总数）；
- 对于无序数据集它被设置成1（被用来检查一个数据集是有序还是无序）。

**7)DATA**
 指定存储点云数据的数据类型。从0.7版本开始，支持两种数据类型：ascii和二进制。

注意：文件头最后一行（DATA）的下一个字节就被看成是点云的数据部分了，它会被解释为点云数据。