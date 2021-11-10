# pi_math

## 目录结构

+ graph: 图算法
+ color：颜色，Vec3/Vec4 的 封装
+ cg2d：2D几何类型
    + geo2d: 基本形状：点，线段，带 孔 多边形；
    + boolean：多边形 布尔运算
+ triangulation：三角剖分，被 cg2d 使用
+ polygon：渐变，圆角矩形 的 三角化；
+ rect_map：二维 动态 装箱算法；
+ spatialtree 空间数据结构：四叉 & 八叉
+ `TODO:` hash_value 计算hash值，建立 jira 任务，让 @罗彬 移到 pi_lib/

## 推荐使用 的 数学 & 物理 库 

#### [rust-num系列：数字 抽象](https://github.com/rust-num/num)

+ [通用Trait：num-traits](https://github.com/rust-num/num-traits)
+ [实现 Range 迭代器：num-iter](https://github.com/rust-num/num-iter)
+ [整数：num-integer](https://github.com/rust-num/num-integer)
+ [大整数：num-bigint](https://github.com/rust-num/num-bigint)
+ [有理数：num-rational](https://github.com/rust-num/num-rational)
+ [复数：num-complex](https://github.com/rust-num/num-complex)

#### 其他

+ [approx: 浮点近似相等](https://github.com/brendanzab/approx)
+ [fixed: 定点数实现](https://crates.io/crates/fixed)
+ [typenum: 编译时数学运算](https://github.com/paholg/typenum)

#### [dimforge系列](https://dimforge.com/)

+ [simba: 抽象代数库](https://github.com/dimforge/simba)
+ [nalgebra: 线性代数库](https://github.com/dimforge/nalgebra)
+ [Parry: 碰撞库](https://github.com/dimforge/parry)
+ [Rapier: 物理库](https://github.com/dimforge/rapier)
+ [Salva: 流体力学库](https://github.com/dimforge/salva)