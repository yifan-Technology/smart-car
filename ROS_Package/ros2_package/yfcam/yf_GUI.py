import rclpy
import yaml
import numpy as np
import time
import tkinter as tk
import tkinter.messagebox
import cv2
import PIL.Image, PIL.ImageTk
import yf_node

class GUI():
    root = tk.Tk()
    var_ObjectNum = tk.StringVar()
    Var_SollSpeed1 = tk.StringVar()
    Var_RealSpeed1 = tk.StringVar()
    Var_SollSpeed2 = tk.StringVar()
    Var_RealSpeed2 = tk.StringVar()
    Var_SollSpeed3 = tk.StringVar()
    Var_RealSpeed3 = tk.StringVar()
    Var_SollSpeed0 = tk.StringVar()
    Var_RealSpeed0 = tk.StringVar()
    def __init__(self,flag,soll):
        [flagSub,flagPub] = flag
                # Base size
        normal_width = 3000
        normal_height = 2000

        # Get screen size
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()

        # Get percentage of screen size from Base size
        percentage_width = screen_width / normal_width
        percentage_height = screen_height / normal_height

        self.text_width_1 = int(15*percentage_width)
        self.text_width_2 = int(30*percentage_width)
        self.ObjectNum_width_1 = int(30*percentage_width)
        self.Scale_width_1 = int(15*percentage_height)
        self.Scale_length_1 = int(2400*percentage_width)


        self.root.resizable(True, True)


        self.nodeFlagSub = flagSub  
        self.nodeFlagPub = flagPub 
        self.nodeSoll = soll         
        fakeImg = np.random.random([320,640,3])*255
        photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(np.uint8(fakeImg)).convert('RGB'))
        
        
        self.sollspeed = [0.0, 0.0, 0.0, 0.0]
        self.realspeed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.liveVideo = photo
        self.costMap = photo    
        
        self.videoLabel = tk.Label(self.root, image=self.liveVideo)
        self.videoLabel.grid(row=1, column=1, padx=2, pady=2, columnspan=5, rowspan=1)
        self.costMapLabel = tk.Label(self.root, image=self.costMap)
        self.costMapLabel.grid(row=1, column=6, padx=2, pady=2, columnspan=5, rowspan=1)

        self.speed = 0
        self.rotate = 0      
        
        self.nodeFlagPub.publishMsg(101)
        self.targetIdx = memoryview(bytes([101]))
        self.entryMsg = None


        self.set_Button()
        self.set_Text()
        self.set_Scale()


        tk.Entry(self.root, textvariable=self.var_ObjectNum,  bg='green', width=self.ObjectNum_width_1
            ).grid(row=5, column=3, padx=2, pady=2, columnspan=2, rowspan=1)



    def set_Scale(self):
		# Scale in column 4
        tk.Scale(self.root, orient='horizontal', from_=0, to=1200, width=self.Scale_width_1, length=self.Scale_length_1, tickinterval=100, command=self.set_speed
            ).grid(row=6, column=2, padx=2, pady=2, columnspan=12, rowspan=1)
        tk.Scale(self.root, orient='horizontal', from_=0, to=250, width=self.Scale_width_1, length=self.Scale_length_1, tickinterval=100, command=self.set_rotate
            ).grid(row=7, column=2, padx=2, pady=2, columnspan=12, rowspan=1)

    def updata_vars(self,real):        
        self.Var_SollSpeed0.set(str(self.sollspeed[0]))
        self.Var_SollSpeed1.set(str(self.sollspeed[1]))
        self.Var_SollSpeed2.set(str(self.sollspeed[2]))
        self.Var_SollSpeed3.set(str(self.sollspeed[3]))

        self.Var_RealSpeed0.set(str(self.realspeed[0]))
        self.Var_RealSpeed1.set(str(self.realspeed[2]))
        self.Var_RealSpeed2.set(str(self.realspeed[4]))
        self.Var_RealSpeed3.set(str(self.realspeed[6]))
        
        rclpy.spin_once(real.node,timeout_sec=0.05)
        self.realspeed = real.subMsg
        
        self.entryMsg = self.var_ObjectNum.get()  
        self.targetIdx = self.nodeFlagSub.subMsg.data
        print(self.targetIdx.tolist())
        
    def set_Text(self):
        tk.Label(self.root, text="Speed Type",width=self.text_width_1
            ).grid(row=2, column=1, padx=2, pady=2)
        tk.Label(self.root, text="左前",width=self.text_width_1
            ).grid(row=2, column=2, padx=2, pady=2)
        tk.Label(self.root, text="右前",width=self.text_width_1
            ).grid(row=2, column=3, padx=2, pady=2)
        tk.Label(self.root, text="左后",width=self.text_width_1
            ).grid(row=2, column=4, padx=2, pady=2)
        tk.Label(self.root, text="右后",width=self.text_width_1
            ).grid(row=2, column=5, padx=2, pady=2)
        tk.Label(self.root, text="加速度",width=self.text_width_1
            ).grid(row=6, column=1, padx=2, pady=2)
        tk.Label(self.root, text="转速差",width=self.text_width_1
            ).grid(row=7, column=1, padx=2, pady=2)
        tk.Label(self.root, text="请选择跟踪目标（输入序号）： ",width=self.text_width_2
            ).grid(row=5, column=1, padx=2, pady=2, columnspan=2)

        tk.Label(self.root, text="叠加转速=500，转速差=100，初始sollspeed=[0,0,0,0]。 前进：[500,-500,500,-500], 左转：[450,-550,450,-550]\n左前：[450,-500,450,-550], 重置：[0,0,0,0]。其余选项同理对称。调整完成后，Soll Speed显示对应的改变",width=90
            ).grid(row=2, column=6, padx=2, pady=2, columnspan=5)

        tk.Label(self.root, text="Soll Speed",width=self.text_width_1
            ).grid(row=3, column=1, padx=2, pady=2)
        tk.Label(self.root, text="Real Speed",width=self.text_width_1
            ).grid(row=4, column=1, padx=2, pady=2)
            
        tk.Label(self.root ,textvariable=self.Var_SollSpeed0, bg='yellow', width=self.text_width_1
            ).grid(row=3, column=2, padx=2, pady=2)
        tk.Label(self.root ,textvariable=self.Var_RealSpeed0, bg='yellow', width=self.text_width_1
            ).grid(row=4, column=2, padx=2, pady=2)
        tk.Label(self.root ,textvariable=self.Var_SollSpeed1, bg='yellow', width=self.text_width_1
            ).grid(row=3, column=3, padx=2, pady=2)
        tk.Label(self.root ,textvariable=self.Var_RealSpeed1, bg='yellow', width=self.text_width_1
            ).grid(row=4, column=3, padx=2, pady=2)
        tk.Label(self.root ,textvariable=self.Var_SollSpeed2, bg='yellow', width=self.text_width_1
            ).grid(row=3, column=4, padx=2, pady=2)
        tk.Label(self.root ,textvariable=self.Var_RealSpeed2, bg='yellow', width=self.text_width_1
            ).grid(row=4, column=4, padx=2, pady=2)
        tk.Label(self.root ,textvariable=self.Var_SollSpeed3, bg='yellow', width=self.text_width_1
            ).grid(row=3, column=5, padx=2, pady=2)
        tk.Label(self.root ,textvariable=self.Var_RealSpeed3, bg='yellow', width=self.text_width_1
            ).grid(row=4, column=5, padx=2, pady=2)


    def set_Button(self):
        tk.Button(self.root, text='屏幕\n锁定', command=self.tracking_flag
            ).grid(row=3, column=6, padx=2, pady=2, columnspan=2, rowspan=1)
        tk.Button(self.root, text='电机\n停止', command=self.stopMotor_flag
            ).grid(row=4, column=6, padx=2, pady=2, columnspan=2, rowspan=1)
        tk.Button(self.root, text='重新\n开始', command=self.reset_flag
            ).grid(row=5, column=6, padx=2, pady=2, columnspan=2, rowspan=1)

        tk.Button(self.root, text='\n           开始追踪           \n', command=self.send_objectNum
            ).grid(row=5, column=5, padx=2, pady=2, columnspan=1, rowspan=1)
        
        tk.Button(self.root, text='\n               左前               \n', command=self.left_front
            ).grid(row=3, column=8, padx=2, pady=2)
        tk.Button(self.root, text='\n               前进               \n', command=self.direct_front
            ).grid(row=3, column=9, padx=2, pady=2)
        tk.Button(self.root, text='\n               右前               \n', command=self.right_front
            ).grid(row=3, column=10, padx=2, pady=2)

        tk.Button(self.root, text='\n               左转               \n', command=self.direct_left 
            ).grid(row=4, column=8, padx=2, pady=2)
        tk.Button(self.root, text='\n               重置               \n', command=self.speed_reset 
            ).grid(row=4, column=9, padx=2, pady=2)
        tk.Button(self.root, text='\n               右转               \n', command=self.direct_right
            ).grid(row=4, column=10, padx=2, pady=2)

        tk.Button(self.root, text='\n               左后               \n', command=self.left_back 
            ).grid(row=5, column=8, padx=2, pady=2)
        tk.Button(self.root, text='\n               后退               \n', command=self.direct_back 
            ).grid(row=5, column=9, padx=2, pady=2)
        tk.Button(self.root, text='\n               右后               \n', command=self.right_back 
            ).grid(row=5, column=10, padx=2, pady=2)

    def tracking_flag(self):        
        self.nodeFlagPub.publishMsg(0)
    def reset_flag(self):        
        self.nodeFlagPub.publishMsg(101)
    def stopMotor_flag(self):        
        self.nodeFlagPub.publishMsg(100)

    def send_objectNum(self):
        if self.entryMsg == "":
            tk.messagebox.showerror('错误',"请输入序号！")
            return
        elif not self.entryMsg.isdecimal():
            tk.messagebox.showerror('错误',"请输入正整数！")
            return
        entryMsg = int(self.entryMsg)
        idx = self.targetIdx.tolist()
        if idx == 101:
            tk.messagebox.showerror('错误',"请先点击“屏幕追踪”按钮")
            return
        else:
            if idx < 0:
                maxInput = abs(idx)
                if maxInput < entryMsg:
                    tk.messagebox.showerror('错误',"请输入正确的序号！(1-{})".format(maxInput))
                else:                    
                    self.nodeFlagPub.publishMsg(int(self.entryMsg))
            elif idx == 0:
                tkinter.messagebox.showinfo('提示','请稍后，目标识别中~')
            elif 100 > idx > 0:
                tkinter.messagebox.showwarning('警告','正在追踪中，点击“开始追踪”重新开始')


    def set_speed(self,var):
        self.speed = int(var)
    def set_rotate(self,var):
        self.rotate = int(int(var)/2)

    # direction
    def left_front(self):
        currentSpeed = self.sollspeed
        self.sollspeed = [currentSpeed[0]-self.rotate,currentSpeed[1],currentSpeed[2]-self.rotate,currentSpeed[3]]
        self.nodeSoll.publishMsg(self.sollspeed)        
        rclpy.spin_once(self.nodeSoll.node,timeout_sec=0.01)
    def direct_front(self):
        currentSpeed = self.sollspeed  
        self.sollspeed = [currentSpeed[0]+self.speed,currentSpeed[1]+self.speed,currentSpeed[2]+self.speed,currentSpeed[3]+self.speed]
        self.nodeSoll.publishMsg(self.sollspeed)        
        rclpy.spin_once(self.nodeSoll.node,timeout_sec=0.01)
    def right_front(self):
        currentSpeed = self.sollspeed
        self.sollspeed = [currentSpeed[0],currentSpeed[1]-self.rotate,currentSpeed[2],currentSpeed[3]-self.rotate]
        self.nodeSoll.publishMsg(self.sollspeed)        
        rclpy.spin_once(self.nodeSoll.node,timeout_sec=0.01)
    

    def direct_left(self):
        currentSpeed = self.sollspeed
        self.sollspeed = [currentSpeed[0]-self.rotate,currentSpeed[1]+self.rotate,currentSpeed[2]-self.rotate,currentSpeed[3]+self.rotate]
        self.nodeSoll.publishMsg(self.sollspeed)        
        rclpy.spin_once(self.nodeSoll.node,timeout_sec=0.01)
    def speed_reset(self):
        self.sollspeed = [0.0,0.0,0.0,0.0]
        self.nodeSoll.publishMsg(self.sollspeed)        
        rclpy.spin_once(self.nodeSoll.node,timeout_sec=0.01)
    def direct_right(self):
        currentSpeed = self.sollspeed
        self.sollspeed = [currentSpeed[0]+self.rotate,currentSpeed[1]-self.rotate,currentSpeed[2]+self.rotate,currentSpeed[3]-self.rotate]
        self.nodeSoll.publishMsg(self.sollspeed)        
        rclpy.spin_once(self.nodeSoll.node,timeout_sec=0.01)
        
    def left_back(self):   
        currentSpeed = self.sollspeed     
        self.sollspeed = [currentSpeed[0]+self.rotate,currentSpeed[1],currentSpeed[2]+self.rotate,currentSpeed[3]]
        self.nodeSoll.publishMsg(self.sollspeed)        
        rclpy.spin_once(self.nodeSoll.node,timeout_sec=0.01)
    def direct_back(self):
        currentSpeed = self.sollspeed
        self.sollspeed = [currentSpeed[0]-self.speed,currentSpeed[1]-self.speed,currentSpeed[2]-self.speed,currentSpeed[3]-self.speed]
        self.nodeSoll.publishMsg(self.sollspeed)        
        rclpy.spin_once(self.nodeSoll.node,timeout_sec=0.01)
    def right_back(self):
        currentSpeed = self.sollspeed
        self.sollspeed = [currentSpeed[0],currentSpeed[1]+self.rotate,currentSpeed[2],currentSpeed[3]+self.rotate]
        self.nodeSoll.publishMsg(self.sollspeed)        
        rclpy.spin_once(self.nodeSoll.node,timeout_sec=0.01)


def init():    
    # 全局化节点名称
    global nodeName  
    # 读取yaml文件
    with open("/home/yf/yifan/config.yaml","r") as f:
        config=yaml.load(f)  
    # 读取节点名称参数
    nodeName = config["RosTopic"]

def cv2PIL(imgMsg,label):#size=(672,376)
    img = imgMsg.subMsg[:,:,0:3][:,:,::-1]
    #img = cv2.resize(img,size)
    pilImg = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(np.uint8(img)).convert('RGB'))
    label.configure(image=pilImg)
    label.image = pilImg

def main(args=None):
    init()
    rclpy.init()  

    video = yf_node.YF_Image_PY(nodeName['Video'],'Video',"sub")     
    showMap = yf_node.YF_Image_PY(nodeName["ShowMap"],"ShowMap","sub") #YF_CompressedImage
    flagSub = yf_node.YF_ObjectFlag(nodeName['ObjectFlag'],'ObjectFlagSub',"sub")
    flagPub = yf_node.YF_ObjectFlag(nodeName['ObjectFlag'],'ObjectFlagPub',"pub")
    real = yf_node.YF_RealSpeed(nodeName['RealSpeed'],'RealSpeed',"sub")
    soll = yf_node.YF_SollSpeed(nodeName['SollSpeed'],'SollSpeed',"sub")
    
    gui = GUI([flagSub,flagPub],soll) 

    t = time.time()
    while True:
        rclpy.spin_once(video.node,timeout_sec=0.01)
        rclpy.spin_once(showMap.node,timeout_sec=0.01)
        rclpy.spin_once(flagSub.node,timeout_sec=0.001)
        rclpy.spin_once(flagPub.node,timeout_sec=0.001)   

        if video.subMsg is None:
            print("Waiting for video")
            continue 
        if showMap.subMsg is None:
            print("Waiting for showMap")
            continue
        
        # print fps
        print("fps: ", int(1/(time.time()-t)))        
        t = time.time() 
        
        cv2PIL(video,gui.videoLabel)
        cv2PIL(showMap,gui.costMapLabel)
        gui.updata_vars(real)
        GUI.root.update()
    
    # 杀死所有订阅节点
    showMap.node.destroy_node()
    flagSub.node.destroy_node()
    flagPub.node.destroy_node()
    soll.node.destroy_node()
    real.node.destroy_node()
    video.node.destroy_node()    
    # 结束rclpy
    rclpy.shutdown()


        
