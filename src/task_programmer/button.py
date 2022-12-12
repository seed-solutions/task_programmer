#!/usr/bin/env python
# -*- coding: utf8 -*-
import tkinter
import subprocess

# not used in case of lib----
import threading
import time

class Button():
  def __init__(self):
    self.running = True
    self.wp = dict()
    self.wp_number = 0
    print("button load")

  def start(self):
    self.running = True
    self.root = tkinter.Tk()
    self.root.attributes("-topmost",True) # 最前面に表示
    
    # 画面サイズの設定
    window_width = 960
    window_height = 540
    self.root.geometry('%dx%d' % (window_width, window_height))
    
    self.root.title('待機中') # 画面タイトルの設定

    #出現位置を画面中央にする
    x = self.root.winfo_screenwidth()/2 - window_width/2
    y = self.root.winfo_screenheight()/2 - window_height/2
    self.root.geometry('+%d+%d' % (x,y))

    # ラベルの作成
    label = tkinter.Label(self.root,text='下のボタンを\n押してください',font=("",50))
    label.pack() #ウインドウにラベルを配置する。

    # ボタンの作成
    btn = tkinter.Button(self.root, text='開始',
      font=("",70),width=15,bg="#80ff80",command=self.quit)
    btn["bd"] = "7"
    btn["relief"] = "groove"
    btn.place(relx=0.5, rely=0.5, anchor=tkinter.CENTER)
    btn.pack()

    # 画面をそのまま表示
    self.running = True
    self.root.after(1000,self.check_quit)
    self.root.protocol("WM_DELETE_WINDOW", self.quit) # 閉じるボタンを押したときのイベント
    self.root.mainloop()

    # 割り込み用にdeleteしておく
    del label
    del btn
    del self.root

  def select_wp(self):
    print("button start")
    wp_list = sorted(self.wp.keys())

    self.running = True
    self.root = tkinter.Tk()
    self.root.attributes("-topmost",True) # 最前面に表示

    self.root.title('待機中') # 画面タイトルの設定

    #全画面にて、出現位置を画面中央にする
    window_width = self.root.winfo_screenwidth()
    window_height = self.root.winfo_screenheight()
    x = self.root.winfo_screenwidth()/2 - window_width/2
    y = self.root.winfo_screenheight()/2 - window_height/2
    self.root.geometry('%dx%d' % (window_width, window_height))
    self.root.geometry('+%d+%d' % (x,y))

    frame1 = tkinter.Frame(self.root)
    frame1.pack(side=tkinter.TOP)

    #ラベルを生成する。
    label = tkinter.Label(frame1,text='行先を選択してください',font=("",50))#,padx=100) 
    label.pack(side=tkinter.LEFT) #ウインドウにラベルを配置する。

    btn_kill = tkinter.Button(frame1, text='終了',activebackground='#ff8080',
      font=("",30),width=5,bg="#ff8080",command=self.quit)
    btn_kill["bd"] = "7"
    btn_kill["relief"] = "groove"
    btn_kill.pack(side=tkinter.RIGHT)

    #ボタン行数の決定
    c_size = 4
    r_size = int(len(wp_list)/c_size + 1)

    #-------------- スクロールバー追加  ---------------
    # Canvas Widget を生成
    canvas = tkinter.Canvas(self.root,width=int(window_width),height=int(window_height))
    # Scrollbar を生成して配置
    bar = tkinter.Scrollbar(self.root, orient=tkinter.VERTICAL, width=100)
    bar.pack(side=tkinter.RIGHT, fill=tkinter.Y)
    # bar.pack(side=tkinter.RIGHT)
    bar.config(command=canvas.yview)
    # Canvas Widget を配置
    canvas.config(yscrollcommand=bar.set)
    canvas.config(scrollregion=(0,0,0, r_size*230)) #スクロール範囲
    # canvas.pack(side=tkinter.LEFT, fill=tkinter.BOTH)
    canvas.pack(side=tkinter.TOP,expand=True)
    # Frame Widgetを 生成
    btn_frame = tkinter.Frame(canvas)
    canvas.create_window((0,0), window=btn_frame, anchor=tkinter.NW, width=canvas.cget('width'))

    # ボタンの作成
    for i in range(r_size):
      for j in range(c_size):
        if(i*c_size+j+1 > len(wp_list)): break
        btn = tkinter.Button(btn_frame, text=wp_list[i*c_size+(j)],activebackground='#80ff80',
          font=("",30),height=2,width=int(window_width/120),bg="#80ff80",wraplength=int(window_width/5))
        btn["bd"] = "7"
        btn["relief"] = "groove"
        btn.bind("<1>",self.set_wp)
        btn.grid(column=j,row=i)

    # 画面をそのまま表示
    self.running = True
    self.root.after(1000,self.check_quit)
    self.root.protocol("WM_DELETE_WINDOW", self.quit) # 閉じるボタンを押したときのイベント
    self.root.mainloop()

    del frame1
    del btn_frame
    del canvas
    del self.root

  def set_wp(self,event):
    self.wp_number = self.wp[event.widget["text"]]
    self.running = False

  def check_quit(self):
    if self.running:
      self.root.after(1000,self.check_quit)
    else:
      self.root.destroy()

  def quit(self):
    self.wp_number = -1
    self.running = False

def main():
  bt = Button()

  for i in range(10):
    bt.wp["p"+str(i)] = i

  thread = threading.Thread(target=bt.select_wp)
  thread.start()

  thread.join()
  print ("wp : ", bt.wp_number)

if __name__ == '__main__':
  main()
