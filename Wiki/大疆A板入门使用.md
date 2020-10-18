# 大疆A板入门使用

## 开发软件前提

- Win10
- JRE：https://www.java.com/en/download/windows-64bit.jsp
- STM32CubeMX：https://www.st.com/en/development-tools/stm32cubemx.html
- Keil MDK：http://www2.keil.com/mdk5
- ST-LINK/V2, ST-LINK/V2-1 USB driver：https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-utilities/stsw-link009.html



### JRE安装成功验证

安装路径保持默认安装路径，安装完毕后在cmd中输入java得到如图结果证明安装成功：

![img](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/a/20934064186/2863267711/d389be86-e70e-4786-be4a-487d54ae1cd1.png)

## 开发硬件前提

- 大疆A板：https://store.dji.com/de/product/rm-development-board-type-a
- stm芯片debugger ST-LINK V2，购买链接：https://www.amazon.de/gp/product/B086TWZNMM/ref=ppx_yo_dt_b_asin_title_o04_s00?ie=UTF8&psc=1

### 开发板与电脑的连接

![img](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/a/20934064186/2863267711/60244277-bcc3-4b7c-8310-8da3e1417422.png)  



## 点亮A板自带的第五颗LED灯

### 打开新项目

![img](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/a/20934064186/2863267711/a7e20556-7d5f-4294-a705-da1f8930f973.png)

### 选择项目对应的开发板 

![img](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/a/20934064186/2863267711/bd1258a3-2b18-4fdf-9790-7713fd91b80a.png)

### 开启Debug模式

以便对开发板烧写程序

![img](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/a/20934064186/2863267711/4b5f5a1f-9ef4-4bbd-b14a-d8873935b331.png)

### 开启时钟

![img](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/a/20934064186/2863267711/d18bfc8d-b33c-4c24-b56b-18dd00344388.png)

### 设置PG5口（对应第五颗LED灯）为GPIO输出口

![img](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/a/20934064186/2863267711/e2698780-986a-40d7-888a-ffe8ea16d471.png)

![img](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/a/20934064186/2863267711/6b4275f0-2b7d-4883-95ee-214001f05f67.png)

参考：[https://rm-static.djicdn.com/tem/RoboMaster%E5%BC%80%E5%8F%91%E7%89%88%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C.pdf](https://rm-static.djicdn.com/tem/RoboMaster开发版用户手册.pdf)

### 时钟设置

![img](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/a/20934064186/2863267711/14ad4ac3-7d83-4ff6-bba2-8a042f2f4a2a.png)

项目名称，开发环境的配置

![img](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/a/20934064186/2863267711/6de49aa7-69a1-4b31-876c-11a188cab87d.png)

### 设置节能模式以及分开头文件![img](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/a/20934064186/2863267711/1c9efa08-aeb5-4549-907f-10d6ede061fc.png) 

### 生成代码并打开项目

点击Open Project后Keil MDK软件会自动开启。

![img](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/a/20934064186/2863267711/f17738af-ea51-4b82-a937-89f1160f14c4.png)

### 配置debugger 

![img](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/a/20934064186/2863267711/9c912578-f15b-435c-8589-f3e548d36971.png)

### Build & Download  

在main.c中添加让LED5每隔1s进行闪烁的代码。然后进行【18】build，成功无报错后点击【19】download。程序就烧写进A板了。之后按下A板的reset按钮，这个A板的hello world就大功告成啦！

![img](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/a/20934064186/2863267711/704ee97b-3083-4319-a684-7f7d0ae77d3f.png) 



​      