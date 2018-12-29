/*
  Author : Siqi Liu (liusiqi@sjtu.edu.cn)
  Date   : 2018.4
*/

//后车car2
#include "includes.h"
int stop_delay=0;
bool stop_delay_flag=0;
// ====== Variables ======
// ---- Global ----
u8 cam_buffer_safe[BLACK_WIDTH*2];
u8 cam_buffer[IMG_ROWS][IMG_COLS+BLACK_WIDTH];   //64*155，把黑的部分舍去是59*128
//通用·赛道识别================================
int MAX_SPEED = 21;
int MIN_SPEED = 16;
float chasu=0.038;
int lap=0;
int cnt_tmp=0;
int cnt_speed=0;
int cnt_speed2=0;
float Dir_Kp=4.4;
float Dir_Kp2=4.4;
float Dir_Kd=32;
float Dir_Kd2=32;
int differ=0;// 记录黑白跳变点
bool stopper=0;
Road road_B[ROAD_SIZE];//由近及远存放
float mid_ave;//road中点加权后的值
float weight[4][10] ={ {0,0,0,0,0,0,0,0,0,0},
{0.500,0.800,1.118,1.454, 2.296, 3.744, 5.304,      6.000, 5.304,3.744},
{0.500,0.800,1.118,1.454, 2.296, 3.744, 5.304,      6.000, 5.304,3.744},
{0.500,0.800,1.118,1.454, 2.296, 3.744, 5.304,      6.000, 5.304,3.744}
//{1.00,1.03,1.14,1.54,2.56,               4.29,6.16,7.00,6.16,4.29},
                        //{1.00,1.03,1.14,1.54,2.56,               4.29,6.16,7.00,6.16,4.29},
                        //{1.00,1.03,1.14,1.54,2.56,               4.29,6.16,7.00,6.16,4.29}
                        //{1.118, 1.454, 2.296, 3.744, 5.304,      6.000, 5.304, 3.744, 2.296, 1.454}
                      };//本来是为直、弯、环岛三个路况分别设置的权重，低速下不用考虑，高速可能会有细微区别。
int valid_row = 0, valid_left = 0, valid_right = 0;//与有效行相关，未有效识别
int valid_row_thr=30;//有效行阈值
u8 car_state=0;//智能车状态标志 0：停止  1：测试舵机  2：正常巡线
u8 remote_state = 0;//远程控制
u8 road_state = 0;//前方道路状态 1、直道   2、弯道  3、环岛  4、障碍 5、十字
                  //2 状态下减速
int margin=30;//弯道判断条件
//环岛处理========================================
int CAM_HOLE_ROW=27; //用来向两边扫描检测黑洞·环岛的cam_buffer行位置     //不用
int check_farthest=20;  //双线延长检测黑洞存在时，最远检测位置，cam_buffer下标，越小越远，不可太小，待调参………………
                        
int check_near=10;//用于观察较近处的路宽判断是否会有分道，road_B下标，越小越近，其值与road_width_thr锁定，待调参………………
                  //对应路宽 5 -> 直道50+ or 弯道70+ or 入环岛或十字110+ or 出环岛可能80~100+ 
int road_width_thr=90;//该值通过观察check_near对应行的正常路宽来确定
int time_cnt=0;
//以下出岛时需要全部置零
int roundabout_state=0;//0-非环岛 1-预入环岛（直道） 2-入环岛（转向） 3-在环岛 4-出环岛（转向）      注：非零的时候会锁定环岛状态
int roundabout_choice=0;//0-未选择 1-左 2-右 3-左右皆可(不用)

int pixinwhite=0;
int frontwhite=0;
int frontblack=0;
int isblackrec;
int iswhitefour;
int TIMEFORHOLE=300;

int cnt_miss=0; //累计未判断成环岛的次数
bool former_choose_left=0,former_choose_right=0;//1=choose 0=not choose
bool is_cross=0; //判断是否是十字
bool jump_miss=0; // 记录连续未检测到拐点的次数
int forced_turn=0;
int jump_thr=20;//两个跳变点检测的阈值
int jump[2][2];//存拐点坐标 0左 1右 0-x 1-y
bool flag_left_jump=0,flag_right_jump=0;
//Hole hole;

//双车处理=======================
int car_type=1; // 后车
bool flag_stop=0;
int stop_time=2;
int overtake_state=0;
bool bt_stop=0;
//观察·速控========================================================
float motor_L;//=MIN_SPEED;
float motor_R;//=MIN_SPEED;
float max_speed;//=MAX_SPEED;
float min_speed;//=MIN_SPEED;

//OLED调参
int debug_speed=0;
PIDInfo debug_dir;

//为缓冲取平均而设置：
int left[DEPTH][ROAD_SIZE];
int right[DEPTH][ROAD_SIZE];
int k_depth=0;

//circle C;
//int c1=15, c2=10, c3=5;

// ---- Local ----
u8 cam_row = 0, img_row = 0;
/*
//——————透视变换·变量——————
double matrix[4][4];
//int buffer[64][128];=cam_buffer
int former[8200][4];
int later[8200][4];
u8 cam_buffer2[64][128];//int buffer2[64][128];
int visited[64][128];

//——————透视变换·函数——————

void getMatrix(double fov, double aspect, double zn, double zf){
    matrix[0][0] = 1 / (tan(fov * 0.5) *aspect) ;
    matrix[1][1] = 1 / tan(fov * 0.5) ;
    matrix[2][2] = zf / (zf - zn) ;
    matrix[2][3] = 1.0;
    matrix[3][2] = (zn * zf) / (zn - zf);
    return;
}

void linearization(){
    int cnt = 0;
    for (int i=0;i<64;i++){
        for (int j=0;j<128;j++){
            former[cnt][0] = i;
            former[cnt][1] = j;
            former[cnt][2] = cam_buffer[i][j];//buffer[i][j];
            former[cnt][3] = 1;
            cnt++;
        }
    }
    return;
}

void multiply(int k){
    for (int i=0;i<4;i++){
        later[k][i] = former[k][0]*matrix[0][i]+former[k][1]*matrix[1][i]+former[k][2]*matrix[2][i]+former[k][3]+matrix[3][i];
    }
    return;
}

void matrixMultiply(){
    for (int i=0;i<8192;i++){
        multiply(i);
    }
    return;
}

void getNewBuffer(){
    for (int i=0;i<64;i++){
        for (int j=0;j<128;j++){
            cam_buffer2[i][j] = 0;
        }
    }
    for (int i=0;i<8192;i++){
        cam_buffer2[later[i][0]][later[i][1]] = later[i][2];
    }
    return;
}
*/

// ====== 

void Cam_Algorithm(){
  static u8 img_row_used = 0;
  
  while(img_row_used ==  img_row%IMG_ROWS); // wait for a new row received
  
  // -- Handle the row --  
  
  if (img_row_used >= BLACK_HEIGHT) {     //前5行黑的不要
    for (int col = 0; col < IMG_COLS; col++) {
      u8 tmp = cam_buffer[img_row_used][col];
      if(!SW1()) UART_SendChar(tmp < 0xfe ? tmp : 0xfd);
    }
   if(!SW1()) UART_SendChar(0xfe);//0xfe->纯参数读取溢出
  }
  
  //  -- The row is used --
  img_row_used++;
  if (img_row_used == IMG_ROWS) {    //一帧图像完行归零，控制算法启动，进入AI_Run
    img_row_used = 0;

    if(!SW1()) UART_SendChar(0xff);//0xff->异常结束
  }//以上原来是SW1()
}

int changdu()
{
  int i;
  for (i=60;i>=0;i--)
  {if (cam_buffer[i][CAM_WID/2]<thr) break;}
  return i;
}

int changdu_left()
{
  int i;
  for (i=60;i>=0;i--)
  {if (cam_buffer[i][10]<thr) break;}
  return i;
}

int changdu_right()
{
  int i;
  for (i=60;i>=0;i--)
  {if (cam_buffer[i][118]<thr) break;}
  return i;
}

int recorddiffer(int row)
{
  int num=0;
  for (int i=0;i<CAM_WID-1;i+=2)
  {
    if (cam_buffer[row][i]<thr&&cam_buffer[row][i+1]>thr||cam_buffer[row][i]>thr&&cam_buffer[row][i+1]<thr) num++;
  }
  return num;
  
}

float constrain(float lowerBoundary, float upperBoundary, float input)
{
  if (input > upperBoundary)
    return upperBoundary;
  else if (input < lowerBoundary)
    return lowerBoundary;
  else
    return input;
}

int constrainInt(int lowerBoundary, int upperBoundary, int input)
{
  if (input > upperBoundary)
    return upperBoundary;
  else if (input < lowerBoundary)
    return lowerBoundary;
  else
    return input;
}
/*
circle getR(float x1, float y1, float x2, float y2, float x3, float y3)
{
  double a,b,c,d,e,f;
  double r,x,y;
  
  a=2*(x2-x1);
  b=2*(y2-y1);
  c=x2*x2+y2*y2-x1*x1-y1*y1;
  d=2*(x3-x2);
  e=2*(y3-y2);
  f=x3*x3+y3*y3-x2*x2-y2*y2;
  x=(b*f-e*c)/(b*d-e*a);
  y=(d*c-a*f)/(b*d-e*a);
  r=sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1));
  x=constrain(-1000.0,1000.0,x);
  y=constrain(-1000.0,1000.0,y);
  r=constrain(1.0,500.0,r);
  bool sign = (x>0)?1:0;
  circle tmp = {r,sign};
  return tmp;
}
*/
bool is_stop_line(int target_line)//目测并不有效……
{
  if((road_B[target_line].right-road_B[target_line].left)<ROAD_WID)
    return 1;
  else return 0;
}
/*
double getSlope_(int x1, int y1, int x2, int y2)
{
  double dx = x2-x1;
  double dy = y2-y1;
  if(dy==0) return dx*100;
  else return (double)dx/dy;
}
*/

void Cam_B_Init()//初始化Cam_B
{
  int i=0;
  for(i=0;i<ROAD_SIZE;i++)
  {
    road_B[i].left=CAM_WID/2;
    road_B[i].right=CAM_WID/2+2;
    road_B[i].mid=CAM_WID/2+1;
  }
  mid_ave=CAM_WID/2+1;
  //以下为road->mid加权值weight的初始化，由近到远
  //方案一：分段函数
  /*for(i=0;i<3;i++)
  {  
    weight[i]=1;
  }
  for(i=3;i<7;i++)
  {  
    weight[i]=2;
  }
  for(i=7;i<10;i++)
  {
    weight[i]=1;
  }*/
  
  //方案二：遵从正态分布，最高值在weight[MaxWeight_index]，在头文件定义相关参数//但是……无效……不知为何
/*  for(int i=0;i<10;i++)
  {
    weight[i]=1.0 + MaxWeight * exp(-(double)exp_k*pow((double)(i-MaxWeight_index),2.0)/2.0); //目前最高下标为常量
  }*/
  
  // design 3 ——>声明与定义放一块，global
//  weight = {1.118, 1.454, 2.296, 3.744, 5.304, 6.000, 5.304, 3.744, 2.296, 1.454};
  
}


bool findblackrec()
{
    int left=CAM_WID/2-CenterRows;
    int right=CAM_WID/2+CenterRows;
    int up=30-CenterLines;
    int down=30+CenterLines;
    for (int i=up;i<down;i++)
    {
      if (cam_buffer[i][left]>thr)  return 0;
    }
    for (int i=up;i<down;i++)
    {
      if (cam_buffer[i][right]>thr) return 0;
    }
    for (int j=left;j>right;j++)
    {
      if (cam_buffer[up][j]>thr) return 0;
    }
    for (int j=left;j>right;j++)
    {
      if (cam_buffer[down][j]>thr) return 0;
    }
    return 1;
}
  

bool whitefour()
{
    int left=CAM_WID/2-CenterRows;
    int right=CAM_WID/2+CenterRows;
    int up=30-CenterLines;
    int down=30+CenterLines;
    bool flag1=1,flag2=0,flag3=0,flag4=0;

    for (int i=down;i<62;i++)
    {
      if (cam_buffer[i][left]>thr&&cam_buffer[i][right]>thr)
      {
        flag2=1;break;
      }
    }
    for ( int j=left;j>0;j--)
    {
      if (cam_buffer[up][j]>thr&&cam_buffer[down][j]>thr)
      {
        flag3=1;break;
      }
    }
    for ( int j=right;j<CAM_WID;j++)
    {
      if (cam_buffer[up][j]>thr&&cam_buffer[down][j]>thr)
      {
        flag4=1; break;
      }
    }
    if (flag1&&flag2&&flag3&&flag4) return 1;
    else return 0;
    
}

bool is_hole(int row)
{
  int left=0,right=0;
    if(cam_buffer[row][CAM_WID/2]<thr)
    {
      //left
      int i=CAM_WID/2-1;
      while(i>0){
        if(left==0 && cam_buffer[row][i]>thr){//是否考虑取平均防跳变？
          left++;
        }
        else if(left==1 && cam_buffer[row][i]<thr){
          left++;
        }
        i--;
      }
      //right
      i=CAM_WID/2+1;
      while(i<CAM_WID){
        if(right==0 && cam_buffer[row][i]>thr){//是否考虑取平均防跳变？
          right++;
        }
        else if(right==1 && cam_buffer[row][i]<thr){
          right++;
        }
        i++;
      }
    }
   // bool static hole=0;
    if(left>=1 && right>=1)
      return 1;
    else return 0;
        
}

bool isWider(int row)
{
  int wid=0;
  for(int i=-1;i<2;i++)
    wid+=(road_B[row+i].right-road_B[row+i].left);
  wid /= 3;
  if(wid>road_width_thr)
    return 1;
  else return 0;
  
}

//test for slope method:
//double theta,theta_d,slope,test;
//double x,y;

  //第一次进化版巡线程序
void Cam_B(){
  //只下载一次：
  /*
  MAX_SPEED_=21;
  MIN_SPEED_=16;
  ROUND_SPEED=10;
  STOP_TIME=150;
  OVERTAKE_SW=1;
  DIR_KP=44;
  DIR_KD=32;
  CHASU=38;
  */
  
  MAX_SPEED=MAX_SPEED_;
  MIN_SPEED=MIN_SPEED_;
  chasu = CHASU/1000;
    Dir_Kp = DIR_KP/10;
    Dir_Kd = DIR_KD;
  
    //===================变量定义====================
    static int dir;//舵机输出  

    
    //================================透视变化
    //getMatrix(0.785398,1.0,1.0,1000);
   // linearization();
   // matrixMultiply();
   // getNewBuffer();
 
    //==================================获取road_B的left right mid 坐标和slope_
    //斜率方案
    /*
    for(int j=0;j<ROAD_SIZE;j++)//从下向上扫描
    {
      //double x,y;
      slope=road_B[j].slope_;
      theta=atan(road_B[j].slope_);//double theta=atan(road_B[j].slope_);
      test=sin(theta);
      test=cos(theta);
      test=tan(theta);
      theta_d=theta*180/PI;
      test=sin(theta_d);//以上用来检测atan函数以及sin函数使用的是否正确，debug模式下
      //left
      x=road_B[j].mid[0];
      y=road_B[j].mid[1];
      while(x>=0 && x<=CAM_WID && y<=CAM_NEAR && y>=CAM_FAR)
      {
        if(cam_buffer[(int)y][(int)x]<thr)
          break;
        else
        {
          x-=cos(theta);
          y-=sin(theta);
        }
      }
      road_B[j].left[0]=constrain(0,CAM_WID,x);
      road_B[j].left[1]=constrain(CAM_FAR,CAM_NEAR,y);

      //right
      x=road_B[j].mid[0];
      y=road_B[j].mid[1];
      while(x>=0 && x<=CAM_WID && y<=CAM_NEAR && y>=CAM_FAR)
      {
        if(cam_buffer[(int)y][(int)x]<thr)
          break;
        else
        {
          x+=cos(theta);
          y+=sin(theta);
        }
      }
      road_B[j].right[0]=constrain(0,CAM_WID,x);
      road_B[j].right[1]=constrain(CAM_FAR,CAM_NEAR,y);
      //mid
      road_B[j].mid[0] = (road_B[j].left[0] + road_B[j].right[0])/2;
      road_B[j].mid[1] = (road_B[j].left[1] + road_B[j].right[1])/2;//分别计算并存储25行的mid
      //slope
      if(j>0)
      {
        road_B[j].slope_=getSlope_(road_B[j-1].mid[0],-road_B[j-1].mid[1],road_B[j].mid[0],-road_B[j].mid[1]);
        road_B[j+1].slope_=road_B[j].slope_;//用以预测下一个mid
      }
      //mid of the next road_B[]
      if(j<(ROAD_SIZE-1))
      {
        road_B[j+1].mid[0]=road_B[j].mid[0]+CAM_STEP*sin(theta);
        road_B[j+1].mid[1]=road_B[j].mid[1]-CAM_STEP*cos(theta);
      }
    }
    */

    
 //   k_depth++;
 //   k_depth%=DEPTH;
    //横向扫描方案============================================================
    for(int j=0;j<ROAD_SIZE;j++)//从下向上扫描
    {
      int i;
      //left
      for (i = road_B[j].mid; i > 0; i--){
        if (cam_buffer[60-CAM_STEP*j][i] < thr)
          break;
        }
      left[k_depth][j]=i;
    //  if(k_depth==DEPTH-1){     //加上这个条件可能会使反应变得不灵敏？？？
  /*      for(int k=0;k<DEPTH;k++)
          road_B[j].left += left[k][j];
        road_B[j].left /= DEPTH;        */
     // }
      road_B[j].left = i;
      
      //right
      for (i = road_B[j].mid; i < CAM_WID; i++){
        if (cam_buffer[60-CAM_STEP*j][i] < thr)
          break;
        }
      right[k_depth][j]=i;
  //    if(k_depth==DEPTH-1){
 /*       for(int k=0;k<DEPTH;k++)
          road_B[j].right += right[k][j];               
        road_B[j].right /= DEPTH;       */
    //  }
      road_B[j].right = i;
      
      //mid
      road_B[j].mid = (road_B[j].left + road_B[j].right)/2;//分别计算并存储共计ROAD_SIZE行(对应cam_buffer第12~60行)的mid
      //store
      if(j<(ROAD_SIZE-1))
        road_B[j+1].mid=road_B[j].mid;//后一行从前一行中点开始扫描
    }
      
    //区分前方道路类型===========================环岛优先级最高
/*    static int mid_ave3;
    static bool flag_valid_row=0;
    for(int i_valid=0;i_valid<(ROAD_SIZE-3) && flag_valid_row==0;i_valid++)     //寻找有效行
    {
      mid_ave3 = (road_B[i_valid].mid + road_B[i_valid+1].mid + road_B[i_valid+2].mid)/3;
      if(mid_ave3<margin||mid_ave3>(CAM_WID-margin))
     // if(road_B[i_valid].mid==road_B[i_valid+1].mid && road_B[i_valid+1].mid==road_B[i_valid+2].mid)
      {
        flag_valid_row=1;
        valid_row=i_valid;
      }
     // else valid_row=ROAD_SIZE-3;
    }
   */
    valid_row = changdu();
    valid_left = changdu_left();
    valid_right = changdu_right();
    if(roundabout_state==0){    //非环岛锁定时，才选择直道或者弯道
      flag_stop=0;
      if(valid_row<valid_row_thr){
        road_state=2;                     //弯道
        //cnt_miss++;
      }
      else {
        road_state=1;                     //直道
        //cnt_miss++;
      }
    }
    
    //检测停止线=====================================================
    
    differ=recorddiffer(57);
    int differ_thr=6;
    
    
    static int stopper=0;
    
    if (differ>differ_thr&&stopper==0)
    {  lap++;stopper=1;}
    
    
    if (differ<differ_thr-2)
      stopper=0;
    
    //累积miss数量清零
    /*
    if (cnt_miss>1000){
      roundabout_flag=0;
      former_choose_left=0;
      former_choose_right=0;
      cnt_miss=0;
      is_cross=0;
    }
    */
    //detect the black hole————————————————————
    /*
   int left=0,right=0;
    if(cam_buffer[CAM_HOLE_ROW][CAM_WID/2]<thr)
    {
      //left
      int i=CAM_WID/2-1;
      while(i>0){
        if(left==0 && cam_buffer[CAM_HOLE_ROW][i]>thr){//是否考虑取平均防跳变？
          left++;
        }
        else if(left==1 && cam_buffer[CAM_HOLE_ROW][i]<thr){
          left++;
        }
        i--;
      }
      //right
      i=CAM_WID/2+1;
      while(i<CAM_WID){
        if(right==0 && cam_buffer[CAM_HOLE_ROW][i]>thr){//是否考虑取平均防跳变？
          right++;
        }
        else if(right==1 && cam_buffer[CAM_HOLE_ROW][i]<thr){
          right++;
        }
        i++;
      }
    }
    bool static hole=0;
    if(left>=1 && right>=1)
      hole=1;//前方有类似黑洞出没
    */
    
    //区分环岛与十字的延长线法如下：
/*    if(roundabout_state==0){     //若没有检测到环岛，则进行拐点（jump）检测，如下：
   // if(1){
      int cnt=0,tmpl1=0,tmpl2=0,tmpr1=0,tmpr2=0;
      double suml=0,sumr=0;
      //int thr_tmp=0;//未用
      flag_left_jump=0,flag_right_jump=0;
      for(cnt=0;cnt<ROAD_SIZE-1;cnt++){
        if(flag_left_jump==0){
          tmpl2=tmpl1;
          tmpl1=road_B[cnt+1].left-road_B[cnt].left;
          suml+=tmpl1;
          if((tmpl2-tmpl1)>jump_thr && cnt>(ROAD_SIZE/5)){      //附加条件：直道长度不能太短，不然偶然性较大
//          if(tmpl1<0&&tmpl2>0) {      //被淘汰的很弱的条件
            flag_left_jump=1;
            jump[0][0]=road_B[cnt].left;
            jump[0][1]=60-CAM_STEP*cnt;
          }
        }
        
        if(flag_right_jump==0){
          tmpr2=tmpr1;
          tmpr1=road_B[cnt+1].right-road_B[cnt].right;
          sumr+=tmpr1;
          if((tmpr1-tmpr2)>jump_thr && cnt>(ROAD_SIZE/5)){      //附加条件：直道长度不能太短，不然偶然性较大
//          if(tmpr1>0&&tmpr2<0) {
            flag_right_jump=1;
            jump[1][0]=road_B[cnt].right;
            jump[1][1]=60-CAM_STEP*cnt;
          } 
        }       
       if(flag_left_jump==1&&flag_right_jump==1)//检测到两个拐点
         break;
      }
      if(flag_left_jump==1&&flag_right_jump==1){//若检测到两个拐点，则判断是环岛还是十字，如下：
        //suml  cnt*CAM_STEP
        int cnt_black_row=0;//标志
        int left_now,right_now;//存当前行扫描边界
        
        for(int j=cnt;(60-CAM_STEP*j)>check_farthest;j++){//最远检测点check_farthest待调参…………………………………………………………
          left_now=jump[0][0]+suml*j/(cnt*CAM_STEP);
          right_now=jump[1][0]+sumr*j/(cnt*CAM_STEP);
          int cnt_black=0;
          for (int i = left_now; i < right_now; i++){
            if (cam_buffer[60-CAM_STEP*j][i] < thr)
              cnt_black++;
          }
          if(cnt_black>(right_now-left_now)*0.8) cnt_black_row++;
          if(cnt_black_row>=3){
       /*     if(is_hole(CAM_HOLE_ROW)){
              road_state=3;                       //完成环岛判断
              roundabout_state=1;
              cnt_miss=0;
            }*/
 /*           break;
          }
          else is_cross=1;
        }
      }
      //出环岛时，若只有一个拐点，出，
      else{
        if (flag_left_jump==1 && is_cross==0){
          former_choose_left==1;
          jump_miss=0;
        }
        if (flag_right_jump==1 && is_cross==0){
          former_choose_right=1;
          jump_miss=0;
        }
        
        //如果未检测到拐点，计数
        if (flag_left_jump==0 && flag_right_jump==0){
          jump_miss++;
        }
      }
    }
*/    //
    
    
    
  
    //detect the obstacle————————————————————
  /*  if((road_B[ROAD_OBST_ROW].right-road_B[ROAD_OBST_ROW].left)<OBSTACLE_THR)
    {
      int i=road_B[ROAD_OBST_ROW].mid;
      left=0;
      right=0;
      //left
      while(i>0){
        if(left==0 && cam_buffer[CAM_HOLE_ROW][i]<thr){
          left++;
        }
        else if(left==1 && cam_buffer[CAM_HOLE_ROW][i]>thr){
          left++;
        }
        else if(left==2 && cam_buffer[CAM_HOLE_ROW][i]<thr){
          left++;
        }
        i--;
      }
      //right
      while(i<CAM_WID){
        if(right==0 && cam_buffer[CAM_HOLE_ROW][i]<thr){
          right++;
        }
        else if(right==1 && cam_buffer[CAM_HOLE_ROW][i]>thr){
          right++;
        }
        else if(right==2 && cam_buffer[CAM_HOLE_ROW][i]<thr){
          right++;
        }
        i++;
      }
      if(left>=3 || right>=3)
        road_state=4;
    }*/
    
    
    static int cnt_hole=0;
    // 1.先检查检测白线部分的算法是否有效
    
    int pixinwhite=0;
    for (int j=0;j<5;j++)
    {
      int white=0;
      for (int i=0;i<CAM_WID;i++)
      {
        if (cam_buffer[j][i]>thr) white++;
      }
      pixinwhite+=white;
    }
    pixinwhite/=5;
    if (pixinwhite>CAM_WID*3/5) frontwhite=1;
    else frontwhite=0;
    
//2.再检查黑块的识别部分的算法是否有效
    
    isblackrec=findblackrec();
//3.除了中间部分的黑块还需要的条件是往两侧还有白块
    
    iswhitefour=whitefour();
    
    if (valid_left < valid_row && valid_right < valid_row && cnt_hole==0/* && valid_left < 15*/) cnt_hole = TIMEFORHOLE;
    
    //=============================根据前方道路类型，进行不同的处理
     switch(road_state)
    {
      case 1: 
        max_speed=MAX_SPEED;
        
       // float static weight1[10] = {1.00,1.03,1.14,1.54,2.56,4.29,6.16,7.00,6.16,4.29};//还未削弱…………………………
        //for(int i=0;i<10;i++) weight[i] = weight1[i];//削弱正态分布程度
        break;
      case 2:
        //max_speed=constrain(MIN_SPEED+1,MAX_SPEED, MAX_SPEED-5);//减多少未定，取决于弯道最高速度
        //float static weight2[10] = {1.00,1.03,1.14,1.54,2.56,4.29,6.16,7.00,6.16,4.29};
        //for(int i=0;i<10;i++) weight[i] = weight2[i];//正态分布的权值
        Dir_Kp=Dir_Kp2;Dir_Kd=Dir_Kd2;
        break;
      case 3:
       // max_speed=constrain(MIN_SPEED+1,MAX_SPEED, MAX_SPEED-5);
/*        max_speed=min_speed+1;
        //float  weight3[10] = {1.118, 1.454, 2.296, 3.744, 5.304, 6.000, 5.304, 3.744, 2.296, 1.454};//未确定
        //for(int i=0;i<10;i++) weight[i] = weight2[i];
        switch(roundabout_state)
        {
        case 0://非环岛，用于置零
          roundabout_state=0;//0-非环岛 1-入环岛（有分支） 2-在环岛 3-出环岛（有分支）
          roundabout_choice=0;//0-未选择 1-左 2-右 3-左右皆可(不用)
          //cnt_miss=0; //累计未判断成环岛的次数
          former_choose_left=0,former_choose_right=0;//1=choose 0=not choose
          //is_cross=0; //判断是否是十字
          //jump_miss=0; // 记录连续未检测到拐点的次数
          forced_turn=0;
          flag_stop=0;
          break;
        case 1:
          if(roundabout_choice==0){
            //暂时用右转代替最短路径（注意：小环岛最短路径影响不大，大环岛能否看到出岛位置则是个问题）
            roundabout_choice=SW1()+1;
          }
          road_width_thr=90;
          if(isWider(check_near)){//如果路过于宽，认为出现分叉，开始转弯
            roundabout_state=2;
            time_cnt=0;
          }
        case 2://入环岛
          //超级限速！！！！
         // max_speed=min_speed+1;
         
          time_cnt++;
           // if(jump_miss>500) forced_turn=roundabout_choice;//是否可行？??????????????????????????
          //  if(jump_miss>1000){
           //   forced_turn=0; 
            //  roundabout_state=2;       //切换到下一个环岛状态
                                        //另一办法是检测分道是否存在，猜想：通过观察较近处的路宽判断是否会有分道
                                        //（尝试如下，在road_B[check_near]处检测，若right-left大于road_width_max（可调参），则利用roundabout_choice将mid_ave左移或右移）
          //  }
            for(int i=1;i<ROAD_SIZE;i++){   //利用roundabout_choice给mid加偏移量
              if(roundabout_choice==1) road_B[i].mid *= 0.25;
              else if(roundabout_choice==2) road_B[i].mid =constrain(0,CAM_WID-1, road_B[i].mid*1.60);
            }
            road_width_thr=100;
            if(!isWider(check_near) && time_cnt>500){ //如果路宽恢复正常，认为完成入岛//^……………………………………………………此处不能脱离状态！！！！！！！！！！！！
              roundabout_state=3;
              time_cnt=0;
            }
            
          
          break;
        case 3://在环岛内，看不到出岛，当做弯道行驶
           //停车：
          if(time_cnt>=100&&time_cnt<(100+stop_time*2200))
            flag_stop=1;
          else flag_stop=0;
          
         
          
          //用来检测什么时候出现分叉//完全不行！！！！！！！！！！！！！！！！！！！！！！！！！！！？？？？？？？？？？？？？？？？？？？？
          time_cnt++;
          if(roundabout_choice==1)
            if(road_B[45].mid<60 && time_cnt>=500){
              roundabout_state=4;
              time_cnt=0;
            }
          else if(roundabout_choice==2)
            if(road_B[45].mid>(CAM_WID-60) && time_cnt>=500){
              roundabout_state=4;
              time_cnt=0;
            }
          if(time_cnt>5000) roundabout_state=0;//约2s  大环岛建议去掉该行
          //如果未检测到，时间又长，说明已经出环岛………………………………如果太短可能会因此而出不了环岛锁定状态……………………………………
            //暂不考虑这种情况，因为大环岛与小环岛用时不同，不可一概而论，（更佳方案是检测纯直道，作为出岛标志）
          break;
        case 4://出环岛，又一次分道
           //停车：
          if(time_cnt>=0&&time_cnt<(stop_time*2200*0.5))
            flag_stop=1;
          else flag_stop=0;
          
          time_cnt++;
          for(int i=1;i<ROAD_SIZE;i+=(ROAD_SIZE/10)){   //利用roundabout_choice给mid加偏移量//与forced_turn异曲同工
            if(roundabout_choice==1) road_B[i].mid = CAM_WID/2-35;
            else if(roundabout_choice==2) road_B[i].mid = CAM_WID/2+35;
          }
          road_width_thr=70;
          if(!isWider(check_near) && time_cnt>=(stop_time*2200+500)){ //如果路宽回复正常，认为出环岛
            roundabout_state=0;
            time_cnt=0;
            flag_stop=0;
          }
          else if(time_cnt>5000) roundabout_state=0;   //2s 未检测到路宽恢复正常则认为出岛
          break;
        default:break;
        }
        
        
        //确定最短路径的一种方法：
        /*
        int left1=road_B[0].left,left2;
        int right1=road_B[0].right,right2;
        int mid1[ROAD_SIZE],mid2[ROAD_SIZE];
        mid1[0]=mid2[0]=road_B[0].mid;
        int mid_branch=CAM_WID/2;
        //检测左右拐点谁先出现
        int tmpl1=0,tmpl2=0,tmpr1=0,tmpr2=0;
        bool flag_branch_choose_left=0,flag_branch_choose_right=0;//0=choose 1=not choose
        
        bool flag_branch=0;
        for(int j=0;j<ROAD_SIZE;j++)//从下向上扫描，重新扫描。如果为提高效率，可以考虑与前一个合并？？
        {
          if(flag_branch==0){
            int i;
            //left
            for (i = mid1[j]; i > 0; i--){
              if (cam_buffer[60-CAM_STEP*j][i] < thr)
                break;
              }
            left1 = i;
            
            //right
            for (i = mid1[j]; i < CAM_WID; i++){
              if (cam_buffer[60-CAM_STEP*j][i] < thr)
                break;
              }
            right1 = i;
            
            //mid
            mid1[j] = (left1 + right1)/2;//分别计算并存储每行的mid
            //next mid
            if(j<(ROAD_SIZE-1))
              mid1[j+1]=mid1[j];//后一行从前一行中点开始扫描
            
            mid2[j]=mid1[j];//copy to mid2
            
            //分道判断
            if(cam_buffer[60-CAM_STEP*j][mid1[j]]<thr){
              flag_branch=1;
              mid_branch=mid1[j];
            }
          }
          else{//开始分道
            
            mid1[j]=(mid_branch+left1)/2;
            mid2[j]=(mid_branch+right1)/2;
            
            int i;
           //left
            if (former_choose_left==1){
            for (i = mid1[j]; i > 0; i--){
              if (cam_buffer[60-CAM_STEP*j][i] < thr)
                break;
              }
            left2=left1;
            left1 = i;
            mid1[j]=(mid_branch+left1)/2;
            }
            
            //right
            if (former_choose_right==1){
            for (i = mid2[j]; i < CAM_WID; i++){
              if (cam_buffer[60-CAM_STEP*j][i] < thr)
                break;
              }
            right2=right1;
            right1 = i;
            mid2[j]=(mid_branch+right1)/2;
            }
            
            if (former_choose_left==0 && former_choose_right==0){
            //计算
            tmpl2=tmpl1;
            tmpl1=left1-left2;
            tmpr2=tmpr1;
            tmpr1=right1-right2;
            
            //检测斜率变化程度
            if(roundabout_choice==0){
              if(tmpl2>0&&tmpl1<=0)
                if((tmpl2-tmpl1)>5){
                  flag_branch_choose_left=1;//choose the left road
                  former_choose_left=1;
                  jump_miss=0;
                  roundabout_choice=1;
                  
                }
              if(tmpr2<0&&tmpr1>=0)
                if((tmpr1-tmpr2)>5){
                  flag_branch_choose_right=1;//choose the right road
                  former_choose_right=1;
                  jump_miss=0;
                  roundabout_choice=2;
                }
              if(flag_branch_choose_left==1&&flag_branch_choose_right==1)
                former_choose_left=1;
                jump_miss=0;
                roundabout_choice=3;
              
            }
            }
          }
          //此处判断flag_branch变化的次数，0-1-0-1-0，然后就可以设为出环岛，从而把roundabout相关量置零
          
        }
        //根据最短路径更新路径中点
        if((flag_branch_choose_left==1 || former_choose_left==1) && jump_miss>400) {
          forced_turn=1;
        }
        else if(flag_branch_choose_right==1 || former_choose_right==1 && jump_miss>400) {
          forced_turn=2;
        }
        if(jump_miss>600){
          forced_turn=0;
          jump_miss=0;
        }
        */
        break;
      case 4:
        break;
      default:break;
    }
    
    //===============================前车变后车
    if (car_type == 0 && overtake_state==1){
      if (waveState == STABLE){
        car_type = 1;
        overtake_state=0;
        cnt_speed=0;
        for(int i=0;i<10;i++)
          UART_SendChar('a');
      }
    }
    
    //================================对十行mid加权：
    float weight_sum=0;
    int step=3;
    for(int j=0;j<10;j++)
    {
      mid_ave += road_B[j*step].mid * weight[road_state][j];
      weight_sum += weight[road_state][j];
    }
    mid_ave/=weight_sum;
    
    //=================================舵机的PD控制
    static float err;
    static float last_err;
    err = mid_ave  - CAM_WID / 2;

    dir = (Dir_Kp+debug_dir.kp) * err + (Dir_Kd+debug_dir.kd) * (err-last_err);     //舵机转向  //参数: (7,3)->(8,3.5)-(3.5,3)
   // if(dir>0)
   //   dir*=1.35;//修正舵机左右不对称的问题//不可删
    last_err = err;
    
//    if (road_state==2)
 //   {
 //     if (dir > 100) dir = 300;
//      else if (dir>100) dir=230;
 //     else if (dir<-100) dir=-300;
//      else if (dir<-180) dir=300;
 //   }
    
    

    while(cnt_hole>0) 
    {
      cnt_tmp = cnt_hole;
      if (car_type == 1 || OVERTAKE_SW==0){
        dir = -250;}
      else dir = 250;
      cnt_hole--;
      if(car_type==0 && cnt_hole==0 && OVERTAKE_SW==1)
        cnt_speed=STOP_TIME;
      else if((car_type==1 && cnt_hole==0) || OVERTAKE_SW==0)
        cnt_speed2=100;
    }
    dir=constrainInt(-250,250,dir);
    
//    if(forced_turn==1) dir=-200;
//    else if(forced_turn==2) dir=200;
    
    if(car_state!=0)
      Servo_Output(dir);
    else   
      Servo_Output(0);
    
    
    
    //==============速度控制=================
    //PWM以dir为参考，前期分级控制弯道速度；中期分段线性控速；后期找到合适参数的时候，再进行拟合——PWM关于dir的函数
    min_speed = MIN_SPEED;
    float range = constrain(0, 50, max_speed - min_speed);//速度范围大小 
    if(flag_stop == 1 || lap>=2)
    {
      if(lap>=2 && stop_delay_flag==0){
        stop_delay_flag=1;
        stop_delay=40;
      }
//     PWM(0, 0, &L, &R);
      if(stop_delay<=0){
        MotorL_Output(0); 
        MotorR_Output(0);
      }
      UART_SendChar('c');
    }
      
    else if(car_state == 2){
      //分段线性控速
/*      if(abs(dir)<50 ){//&& valid_row>valid_row_thr
        motor_L=motor_R=max_speed;
      }
      else if(abs(dir)<95){
        motor_L=motor_R=max_speed-0.33*range*(abs(dir)-50)/45;
        if(dir>0) motor_R=constrain(min_speed,motor_R,motor_R*0.9);//右转
        else motor_L=constrain(min_speed,motor_L,motor_L*0.9);//0.9
      }
      else if(abs(dir)<185){    
        motor_L=motor_R=max_speed-0.33*range-0.33*range*(abs(dir)-95)/90;
        if(dir>0) motor_R=constrain(min_speed,motor_R,motor_R*0.8);//右转
        else motor_L=constrain(min_speed,motor_L,motor_L*0.8);//0/8
      }
      else if(abs(dir)<=300){
        motor_L=motor_R=max_speed-0.66*range-0.33*range*(abs(dir)-185)/45;
        if(dir>0) motor_R=constrain(min_speed,motor_R,motor_R*0.7);//右转
        else motor_L=constrain(min_speed,motor_L,motor_L*0.7);//0.7
      }//以上的差速控制参数未确定，调参时以车辆稳定行驶为目标
      else{
        motor_L=motor_R=min_speed;
      }
      
 */   
//      if (cnt_hole>0) 
//         MIN_SPEED=20;
      
      if (cnt_speed > 0){
        MIN_SPEED=1;
        //MotorL_Output(0);
        //MotorR_Output(0);
        overtake_state=1;
      }
      else if(cnt_speed2>0){
        MIN_SPEED=ROUND_SPEED;
      }
      else {
        MIN_SPEED=MIN_SPEED_;
        overtake_state=0;
      }
      
      if(valid_row < 30)
        {
          motor_L = motor_R = MAX_SPEED;
        }
        else
        {
          motor_L = MIN_SPEED*(1 + chasu * dir / 70);
          motor_R = MIN_SPEED*(1 - chasu * dir / 70);
        }
      if (distance <= 400 && overtake_state == 0 && waveState == STABLE && bt_stop==0){
         motor_L *= 0.7;
         motor_R *= 0.7;
      }
      if (road_state==1 && waveState == STABLE && distance >=600 || bt_stop==1){
         motor_L *= 1.2;
         motor_R *= 1.2;
      }
      
      PWM(motor_L, motor_R, &L, &R);               //后轮速度
      

    }
      
      
      
   else
   {
      MotorL_Output(0); 
      MotorR_Output(0);
   }
   
   
    
    //方案二//暂时放弃
    //C=getR(road_B[c1].mid,20-c1,road_B[c2].mid,20-c2,road_B[c3].mid,20-c3);
    
}




// ====== Basic Drivers ======

void PORTC_IRQHandler(){
  if((PORTC->ISFR)&PORT_ISFR_ISF(1 << 8)){  //CS
    PORTC->ISFR |= PORT_ISFR_ISF(1 << 8);
    if(img_row < IMG_ROWS && cam_row % IMG_STEP == 0 ){
      DMA0->TCD[0].DADDR = (u32)&cam_buffer[img_row][-BLACK_WIDTH];
      DMA0->ERQ |= DMA_ERQ_ERQ0_MASK; //Enable DMA0
      ADC0->SC1[0] |= ADC_SC1_ADCH(4); //Restart ADC
      DMA0->TCD[0].CSR |= DMA_CSR_START_MASK; //Start
    }
    cam_row++;
  }
  else if(PORTC->ISFR&PORT_ISFR_ISF(1 << 9)){   //VS
    PORTC->ISFR |= PORT_ISFR_ISF(1 << 9);
    cam_row = img_row = 0;
  }
}

void DMA0_IRQHandler(){
  DMA0->CINT &= ~DMA_CINT_CINT(7); //Clear DMA0 Interrupt Flag
  
  img_row++; 
}

void Cam_Init(){
  // --- IO ---
  
  PORTC->PCR[8] |= PORT_PCR_MUX(1); //cs
  PORTC->PCR[9] |= PORT_PCR_MUX(1); //vs
  PORTC->PCR[11] |= PORT_PCR_MUX(1);    //oe
  PTC->PDDR &=~(3<<8);
  PTC->PDDR &=~(1<<11);
  PORTC->PCR[8] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(10); //PULLUP | falling edge
  PORTC->PCR[9] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(9);  // rising edge
  PORTC->PCR[11] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK ;
  
  NVIC_EnableIRQ(PORTC_IRQn);
  NVIC_SetPriority(PORTC_IRQn, NVIC_EncodePriority(NVIC_GROUP, 1, 2));
  
  // --- AD ---
  
  /*
  SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;  //ADC1 Clock Enable
  ADC0->CFG1 |= 0
             //|ADC_CFG1_ADLPC_MASK
             | ADC_CFG1_ADICLK(1)
             | ADC_CFG1_MODE(0);     // 8 bits
             //| ADC_CFG1_ADIV(0);
  ADC0->CFG2 |= //ADC_CFG2_ADHSC_MASK |
                ADC_CFG2_MUXSEL_MASK |  // b
                ADC_CFG2_ADACKEN_MASK; 
  
  ADC0->SC1[0]&=~ADC_SC1_AIEN_MASK;//disenble interrupt
  
  ADC0->SC2 |= ADC_SC2_DMAEN_MASK; //DMA
  
  ADC0->SC3 |= ADC_SC3_ADCO_MASK; // continuous
  
  //PORTC->PCR[2]|=PORT_PCR_MUX(0);//adc1-4a
  
  ADC0->SC1[0] |= ADC_SC1_ADCH(4);
  */
  
  SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; //ADC1 Clock Enable
  ADC0->SC1[0] &= ~ADC_SC1_AIEN_MASK; //ADC1A
  ADC0->SC1[0] = 0x00000000; //Clear
  ADC0->SC1[0] |= ADC_SC1_ADCH(4); //ADC1_5->Input, Single Pin, No interrupt
  ADC0->SC1[1] &= ~ADC_SC1_AIEN_MASK; //ADC1B
  ADC0->SC1[1] |= ADC_SC1_ADCH(4); //ADC1_5b
  ADC0->SC2 &= 0x00000000; //Clear all.
  ADC0->SC2 |= ADC_SC2_DMAEN_MASK; //DMA, SoftWare
  ADC0->SC3 &= (~ADC_SC3_AVGE_MASK&~ADC_SC3_AVGS_MASK); //hardware average disabled
  ADC0->SC3 |= ADC_SC3_ADCO_MASK; //Continuous conversion enable
  ADC0->CFG1|=ADC_CFG1_ADICLK(1)|ADC_CFG1_MODE(0)|ADC_CFG1_ADIV(0);//InputClk, ShortTime, 8bits, Bus
  ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK; //ADC1  b
  ADC0->CFG2 |= ADC_CFG2_ADACKEN_MASK; //OutputClock
    
  // --- DMA ---
  
  SIM->SCGC6 |= SIM_SCGC6_DMAMUX_MASK; //DMAMUX Clock Enable
  SIM->SCGC7 |= SIM_SCGC7_DMA_MASK; //DMA Clock Enable
  DMAMUX->CHCFG[0] |= DMAMUX_CHCFG_SOURCE(40); //DMA0->No.40 request, ADC0
  DMA0->TCD[0].SADDR = (uint32_t) & (ADC0->R[0]); //Source Address 0x400B_B010h
  DMA0->TCD[0].SOFF = 0; //Source Fixed
  DMA0->TCD[0].ATTR = DMA_ATTR_SSIZE(0) | DMA_ATTR_DSIZE(0); //Source 8 bits, Aim 8 bits
  DMA0->TCD[0].NBYTES_MLNO = DMA_NBYTES_MLNO_NBYTES(1); //one byte each
  DMA0->TCD[0].SLAST = 0; //Last Source fixed
  DMA0->TCD[0].DADDR = (u32)cam_buffer;
  DMA0->TCD[0].DOFF = 1;
  DMA0->TCD[0].CITER_ELINKNO = DMA_CITER_ELINKNO_CITER(IMG_COLS+BLACK_WIDTH);
  DMA0->TCD[0].DLAST_SGA = 0;
  DMA0->TCD[0].BITER_ELINKNO = DMA_BITER_ELINKNO_BITER(IMG_COLS+BLACK_WIDTH);
  DMA0->TCD[0].CSR = 0x00000000; //Clear
  DMA0->TCD[0].CSR |= DMA_CSR_DREQ_MASK; //Auto Clear
  DMA0->TCD[0].CSR |= DMA_CSR_INTMAJOR_MASK; //Enable Major Loop Int
  DMA0->INT |= DMA_INT_INT0_MASK; //Open Interrupt
  //DMA->ERQ&=~DMA_ERQ_ERQ0_MASK;//Clear Disable
  DMAMUX->CHCFG[0] |= DMAMUX_CHCFG_ENBL_MASK; //Enable
  
  NVIC_EnableIRQ(DMA0_IRQn);
  NVIC_SetPriority(DMA0_IRQn, NVIC_EncodePriority(NVIC_GROUP, 1, 2));
}
