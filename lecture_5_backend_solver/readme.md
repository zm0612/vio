# 实验报告

## 1. 完成单目BA求解器中的代码：Problem::MakeHessian()和Problem::SolveLinearSystem()

### 1.1 经验总结

**代码流程**

首先代码(lecture_5_backend_solver/code/app/TestMonoBA.cpp)的整体逻辑比较清晰，大致流程如下：

- 生成仿真数据：路标点和相机姿态；
- 构建顶点：相机姿态、路标点逆深度；
- 构建边：三元边(两个相机姿态+逆深度)的重投影误差；
- 然后构建优化器进行迭代求解。

**核心的难点**

a. 对于边的雅克比计算，PPT中都已经给出了详细的计算过程，可以参考；

b. 对于Hessian矩阵的构建并不是很难，需要格外注意矩阵的索引，只要索引都正确一般没什么难度；

c. 对于Hessian的求解，可以先利用舒尔补求解Pose对应的增量，然后再计算landmark对应的增量，应为landmark是一个稀疏矩阵。对于landmark对应的Hessian矩阵，可以使用分块求逆；

> 以上过程PPT中给了详细的过程，参考lecture 3和4的PPT比较容易写出来。

### 1.2 实验结果

**a. 不固定任何顶点**

>ordered_landmark_vertices_ size : 20
>iter: 0 , chi= 5.35099 , Lambda= 0.00597396
>iter: 1 , chi= 0.0301337 , Lambda= 0.00199132
>iter: 2 , chi= 0.000848882 , Lambda= 0.00132755
>iter: 3 , chi= 0.000840597 , Lambda= 0.00177006
>iter: 4 , chi= 0.000836098 , Lambda= 0.00236008
>problem solve cost: 0.73899 ms
>makeHessian cost: 0.34461 ms
>
>Compare MonoBA results after opt...
>after opt, point 0 : gt 0.220938 ,noise 0.227057 ,opt 0.220367
>after opt, point 1 : gt 0.234336 ,noise 0.314411 ,opt 0.234588
>after opt, point 2 : gt 0.142336 ,noise 0.129703 ,opt 0.142355
>after opt, point 3 : gt 0.214315 ,noise 0.278486 ,opt 0.213878
>after opt, point 4 : gt 0.130629 ,noise 0.130064 ,opt 0.130118
>after opt, point 5 : gt 0.191377 ,noise 0.167501 ,opt 0.191559
>after opt, point 6 : gt 0.166836 ,noise 0.165906 ,opt 0.166945
>after opt, point 7 : gt 0.201627 ,noise 0.225581 ,opt 0.201705
>after opt, point 8 : gt 0.167953 ,noise 0.155846 ,opt 0.16754
>after opt, point 9 : gt 0.21891 ,noise 0.209697 ,opt 0.218986
>after opt, point 10 : gt 0.205719 ,noise 0.14315 ,opt 0.205696
>after opt, point 11 : gt 0.127916 ,noise 0.122109 ,opt 0.127512
>after opt, point 12 : gt 0.167904 ,noise 0.143334 ,opt 0.167932
>after opt, point 13 : gt 0.216712 ,noise 0.18526 ,opt 0.216217
>after opt, point 14 : gt 0.180009 ,noise 0.184249 ,opt 0.179541
>after opt, point 15 : gt 0.226935 ,noise 0.245716 ,opt 0.227175
>after opt, point 16 : gt 0.157432 ,noise 0.176529 ,opt 0.157117
>after opt, point 17 : gt 0.182452 ,noise 0.14729 ,opt 0.181943
>after opt, point 18 : gt 0.155701 ,noise 0.182258 ,opt 0.155248
>after opt, point 19 : gt 0.14646 ,noise 0.240649 ,opt 0.146331
>------------ pose translation ----------------
>translation after opt: 0 :-0.0126673 -0.0208772  0.0131711 || gt: 0 0 0
>translation after opt: 1 :-1.05976  4.00208 0.864682 || gt:  -1.0718        4 0.866025
>translation after opt: 2 :-3.98528  6.94819 0.861153 || gt:       -4   6.9282 0.866025

执行结果可以发现，优化之后第一个相机的位置发生了轻微的平移。这确实印证了单目BA问题存在零空间。

**b. 固定两个相机位姿顶点**

> 0 order: 0
> 1 order: 6
> 2 order: 12
>
>  ordered_landmark_vertices_ size : 20
> iter: 0 , chi= 5.35099 , Lambda= 0.00597396
> iter: 1 , chi= 0.0321522 , Lambda= 0.00199132
> iter: 2 , chi= 0.00278558 , Lambda= 0.000663774
> iter: 3 , chi= 0.00277605 , Lambda= 0.000442516
> iter: 4 , chi= 0.00277121 , Lambda= 0.000295011
> problem solve cost: 0.36187 ms
>    makeHessian cost: 0.177168 ms
>
> Compare MonoBA results after opt...
> after opt, point 0 : gt 0.220938 ,noise 0.227057 ,opt 0.220595
> after opt, point 1 : gt 0.234336 ,noise 0.314411 ,opt 0.234412
> after opt, point 2 : gt 0.142336 ,noise 0.129703 ,opt 0.142267
> after opt, point 3 : gt 0.214315 ,noise 0.278486 ,opt 0.214268
> after opt, point 4 : gt 0.130629 ,noise 0.130064 ,opt 0.130345
> after opt, point 5 : gt 0.191377 ,noise 0.167501 ,opt 0.191473
> after opt, point 6 : gt 0.166836 ,noise 0.165906 ,opt 0.166913
> after opt, point 7 : gt 0.201627 ,noise 0.225581 ,opt 0.201665
> after opt, point 8 : gt 0.167953 ,noise 0.155846 ,opt 0.167803
> after opt, point 9 : gt 0.21891 ,noise 0.209697 ,opt 0.218787
> after opt, point 10 : gt 0.205719 ,noise 0.14315 ,opt 0.205678
> after opt, point 11 : gt 0.127916 ,noise 0.122109 ,opt 0.127592
> after opt, point 12 : gt 0.167904 ,noise 0.143334 ,opt 0.167877
> after opt, point 13 : gt 0.216712 ,noise 0.18526 ,opt 0.216592
> after opt, point 14 : gt 0.180009 ,noise 0.184249 ,opt 0.179797
> after opt, point 15 : gt 0.226935 ,noise 0.245716 ,opt 0.227095
> after opt, point 16 : gt 0.157432 ,noise 0.176529 ,opt 0.15736
> after opt, point 17 : gt 0.182452 ,noise 0.14729 ,opt 0.182083
> after opt, point 18 : gt 0.155701 ,noise 0.182258 ,opt 0.155373
> after opt, point 19 : gt 0.14646 ,noise 0.240649 ,opt 0.146334
> ------------ pose translation ----------------
> translation after opt: 0 :0 0 0 || gt: 0 0 0
> translation after opt: 1 : -1.0718        4 0.866025 || gt:  -1.0718        4 0.866025
> translation after opt: 2 :-3.98127  6.94912 0.856755 || gt:       -4   6.9282 0.866025

固定两个位姿顶点之后，前两个位姿就不发生变化了，固定住了位姿。

**c. 固定一个相机位姿和一个逆深度**

求解失败！

单目BA问题的零空间为7维，理论上来说固定一个相机位姿，然后再固定一个逆深度，Hessian矩阵就能满秩，零空间就会消失。但是实际操作时，并不能如此。代码中对于顶点数值固定的问题求解，采用的方法是设置其雅克比矩阵为0，那么迭代的时候就不会有增量，但是这就带来了一些问题。正常情况下，如果直接对Hessian矩阵求逆，那么为零的块会出现无穷大，那么此时数值发散无法求解。

当固定两个位姿的时候，采用舒尔补分块求正规方程的时候，那么它们对应的Hessian块为0，但是由于舒尔补的运算marg的是landmark，舒尔补之后Hessian的块不为零了，此时可以正常求逆，不会出现发散。一旦在此种情况去固定一个位姿和一个路标点，那么路标点对应的Hessian块求逆就会数值发散。

如果使用LM算法，由于会增加一个阻尼因子，此时就不会出现Hessian对应快为0的情况，可以正常求解。但是即使阻尼因子较小，但是加进去之后可能也会产生一个更新的增量，使得即使是固定的顶点的结果在零空间变化。

**d. 对于Hessian矩阵直接求逆，不使用边缘化**

求解失败！

由于Hessian矩阵不满秩，无法直接求逆，所以对于正规方程无法直接求解。

## 2. 完成单目BA求解器中的代码：Problem::TestMarginalize()

该代码的编写没什么难度，按照PPT思路填写进去进行。主要的重心应该放在理解边缘化以及舒尔补的操作上。

执行结果如下：

> ---------- TEST Marg: before marg------------
>      100     -100        0
>     -100  136.111 -11.1111
>        0 -11.1111  11.1111
>
> ---------- TEST Marg: 将变量移动到右下角------------
>      100        0     -100
>        0  11.1111 -11.1111
>     -100 -11.1111  136.111
>
> ---------- TEST Marg: after marg------------
>  26.5306 -8.16327
> -8.16327  10.2041

本来变量1和变量3是不相关的，但是由于边缘化掉变量2之后，1和3就受到了牵制，因此就相关了。

## 3. 对比不同操作方法操作Hessian矩阵自由度的给正规方程求解带来的效果

由于单目BA问题有7个自由度(姿态+尺度)，所以其Hessian矩阵的秩会比实际变量个数少7个，此时对于Hessian矩阵而言其不满秩，因此无法直接求取其逆矩阵。

为了解决这个问题，在论文《On the Comparison of Gauge Freedom Handling in Optimization-based Visual-Inertial State Estimation》中总结了目前常用的三种方法：

- 固定顶点：固定某一些优化的顶点，会使得系统本身的状态量数量减少，此时系统就变成完全可观的；
- 增加先验：用一个附加惩罚项，添加到Hessian矩阵中，给系统增加额外的约束，最终使得系统完全可观；
- 无约束优化：不给系统增加任何约束，直接进行求解，系统会由于不可观性，导致整体坐标发生偏移，此时将优化之后的左边再变换到原来的坐标下。

论文中提到，三种方法的得到的最终准确性是一样的。基于增加先验的方法，需要进行实验选择合适的先验。无约束的优化它比其他两种方法快一些，而且它是通用的，但是优化完成之后需要修正零空间的变化。

对于**无约束优化**的方法，如果使用高斯牛顿进行优化，则必须使用舒尔布的方法，避开对姿态求逆(不可逆)。但是使用
LM算法时无需担心矩阵是否可逆的问题，可以直接求解。

对于**增加先验**的方法，可以给想要固定的顶点增加一个先验的对角矩阵，如下，当想要固定第1个顶点的时候：
$$
(H_{11} + \sigma^2 I) \cdot \Delta x = -Jf
$$
在原等式中$H_{11} \cdot \Delta x= -Jf$，此时增加一个变量但是等式依然成立，那么就只能$\Delta x=0$.

**对比先验方法中使用不同的权值给优化结果带来的影响**

|        sigma         |    0.01     |     0.1     |      1      |     10      |     100     |    1000     |    10000    |
| :------------------: | :---------: | :---------: | :---------: | :---------: | :---------: | :---------: | :---------: |
| 第一个位姿的位移误差 | 0.000465136 | 0.000467064 | 0.000449914 | 3.90435e-05 | 5.85163e-07 | 5.90421e-09 | 5.90475e-11 |
| 第二个位姿的位移误差 | 0.000465136 | 0.000467064 | 0.000449914 | 3.90435e-05 | 5.85163e-07 | 5.90421e-09 | 5.90475e-11 |
| 第三个位姿的位移误差 |  0.0118703  |  0.0119233  |  0.0112061  | 0.00506845  | 0.00668031  |  0.0067077  | 0.00670797  |
|       用时(ms)       |    0.55     |    0.68     |    0.531    |    0.647    |    0.666    |  0.857318   |  0.857318   |
|       迭代次数       |      3      |      3      |      3      |      3      |      3      |      3      |      3      |

> 实际上上面的实验做的并不充分，偶然因素的影响较多，这也就说明了，这些方法之间实际上差距并不大。在实际使用时，应该根据不同的代码实现采用上面的三种方法分别尝试找到一个合适的方案。
>
> 但是在测试的过程中，有一点是比较明确的，使用添加先验的方式，其收敛的速度明显优于舒尔布的方式。
