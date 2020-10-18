#  如何使用shell指令，多terminal并行


## 思路
使用shell的Function指令，每个function代表一个对应的terminal。

## .sh文件写法

```shell

// 首行：
 #！bin/bash       //这行告诉系统执行该脚本程序

// echo 语句，示意程序开始
echo “程序开始啦~”
// 定义所有function（terminal）
 function func_name_0(){

 terminal 1 //内容

 } 

 function func_name_1(){

 terminal 1 //内容

 }

 function func_name_2(){

 terminal 1 //内容

 }
 // 并行运行所有function（terminal）
 
 func_name_1 & func_name_2 & func_name_3 
 
 // echo 语句，示意程序结束
echo “程序结束了~”

 // 结束后杀死所有进程
 kill 0
```


