//UTF-8
#define GLUT_DISABLE_ATEXIT_HACK
#define ZVALUE 20.0f
#define PI 3.1415926535897932384626433832795028
#define EPSILON 1e-6
#define EPSILON_NORMAL 1e-2
#define LIGHT_N 15
#define LIGHT_A 0
#define LIGHT_D 0.4
#define LIGHT_S 0.8
#define OBJ_PATH "./assets/lily-impeller.obj" //宏定义需要使用的OBJ文件路径,目前设定为C盘根目录
#define MAP_PATH "./assets/24mip.bmp"

//使用两个外部头文件,实现了向量与矩阵类型
//来源:http://www.songho.ca/opengl/gl_matrix.html
#include "Vectors.h"
#include "Matrices.h"

#include <GLUT.H>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
using namespace std;

const int WIDTH = 1000; //显示窗口宽度像素数
const int HEIGHT = 1000; //显示窗口高度像素数


int FrameBuffer[HEIGHT][WIDTH]; //帧缓冲器,使用int类型存储下一帧图像各像素颜色
int ZBuffer[HEIGHT][WIDTH]; //深度缓冲器,存储显示窗口各像素对应的源图形上点的深度值

//三角面元数据结构
struct Mesh
{
    Vector4 point[3], point_normal[3], normal; //三角面元的三个顶点与法向向量
    Vector2 mipmap[3];  //三个顶点的纹理坐标
    int color; //三角面元的颜色
};

struct Light
{
    Vector4 position;
    int color;
};

//自行实现的渲染管线函数库
int norm(Mesh& mesh);
Vector4 get_vector(Mesh& mesh);
int get_Z(Mesh& mesh, int x, int y);
int clear(int color);
int put_point(int x, int y, int z, int color);
int color_vector2int(Vector3 color);
Vector3 color_int2vector(int color);
int scan(Mesh& mesh);
int scan_batch(vector<Mesh> model);
int flush();
float get_degree(float dy, float dx);
Matrix4 get_eye_transform_matrix(Vector4 position, Vector4 direction, Vector4 upper);
Mesh transform(Mesh mesh, Matrix4 T);
vector<Mesh> transform_batch(vector<Mesh> model, Matrix4 T);
vector<Mesh> obj_loader(const string& path);
vector<vector<Vector3>> mipmap_loader(const string& path);


//使三角面元的三个顶点的比率系数为1
int norm(Mesh& mesh)
{
    for (int i = 0; i < 3; i++)
    {
        mesh.point[i].x /= mesh.point[i].w;
        mesh.point[i].y /= mesh.point[i].w;
        mesh.point[i].z /= mesh.point[i].w;
        mesh.point[i].w = 1;
    }
    return 0;
}

//获得三角面元的法向量并存入其数据结构
Vector4 get_vector(Mesh& mesh)
{
    Vector4 a, b;
    a = mesh.point[1] - mesh.point[0]; //获得面元上的两个非平行向量
    b = mesh.point[2] - mesh.point[0];
    mesh.normal = { a.y * b.z - a.z * b.y,a.z * b.x - a.x * b.z,a.x * b.y - a.y * b.x,0 }; //两个非平行向量叉乘得到法向向量
    return -mesh.normal;
}

//对于三角面元在XOY平面上的任意一点,计算其深度值(即Z轴坐标)
int get_Z(Mesh& mesh, int x, int y)
{
    //将Z设为未知数,拿到端点到该点的向量,与面元法向向量点乘为0,即可解出Z坐标的表达式
    if (abs(mesh.normal.z) < EPSILON_NORMAL) return 0x7FFFFFFF;
    return (int)((mesh.normal.z * mesh.point[0].z + mesh.normal.y * (mesh.point[0].y - y) + mesh.normal.x * (mesh.point[0].x - x)) / mesh.normal.z);
}

//帧缓冲器与深度缓冲器清空函数
int clear(int color)
{
    //对于每一点,将帧缓冲器设为函数参数传入的背景颜色,深度缓冲器置为最大值
    for (int i = 0; i < HEIGHT; i++)
        for (int j = 0; j < WIDTH; j++)
        {
            FrameBuffer[i][j] = color;
            ZBuffer[i][j] = 0x7FFFFFFF;
        }
    return 0;
}

//带有深度测试功能的绘制点函数
int put_point(int x, int y, int z, int color)
{
    if (z > ZBuffer[y][x]) return 1; //若深度大于缓冲器的值即代表比已显示内容远,直接返回不予绘制
    //若小于,更新帧缓冲器颜色与深度缓冲器存储的深度值
    ZBuffer[y][x] = z;
    FrameBuffer[y][x] = color;
    return 0;
}

int color_vector2int(Vector3 color)
{
    return ((int)(color.x * 0xFF) << 16) | ((int)(color.y * 0xFF) << 8) | (int)(color.z * 0xFF);
}

Vector3 color_int2vector(int color)
{
    return Vector3{ (float)((color >> 16) & 0x000000FF) / 0xFF,
                   (float)((color >> 8) & 0x000000FF) / 0xFF,
                   (float)(color & 0x000000FF) / 0xFF };
}

Vector3 light_phong(Vector4 position,int color,Vector4 normal,Vector4 eye,vector<Light> light)
{
    Vector3 K=color_int2vector(color);
    Vector4 V=(eye-position).normalize();
    normal.normalize();
    Vector3 res=LIGHT_A*K;
    for(vector<Light>::iterator i=light.begin();i!=light.end();i++)
    {
        Vector3 I=color_int2vector(i->color);
        Vector4 L=(i->position-position).normalize();
        Vector4 H=((L+V)/2).normalize();
        res+=LIGHT_D*(L.dot(normal)>0?L.dot(normal):0)*K*I+LIGHT_S*(pow((H.dot(normal)>0?H.dot(normal):0),LIGHT_N)*K*I);
    }
    res={min(1.f,max(0.f,res.x)),min(1.f,max(0.f,res.y)),min(1.f,max(0.f,res.z))};
    return res;
}

//三角面元扫描转换函数
int scan(Mesh &mesh,vector<Light> light,Vector4 eye)
{
    get_vector(mesh); //计算面元的法向量
    char m_max = 0,m_min = 1,m_mid;
    int x_min=0x7FFFFFFF,x_max=-0x8000000;
    float x0,x1,d0,d1;
    Vector3 n0,n1,nd0,nd1,nd,n;
    Vector3 c_min,c_mid,c_max;
    for(char i=0;i<3;i++)
    {
        //找到在y轴上最大点与最小点的下标
        if(mesh.point[i].y>mesh.point[m_max].y)
            m_max=i;
        if(mesh.point[i].y<mesh.point[m_min].y)
            m_min=i;
        //找到面元x轴上的最大最小值,用于消除扫描线带来的毛刺现象
        x_min=min(x_min,(int)mesh.point[i].x);
        x_max=max(x_max,(int)mesh.point[i].x);
    }
    m_mid = 3 - m_max - m_min; //计算y轴中间点(即扫描转换分界点)下标
    c_min = light_phong(mesh.point[m_min],mesh.color,mesh.point_normal[m_min],eye,light);
    c_mid = light_phong(mesh.point[m_mid],mesh.color,mesh.point_normal[m_mid],eye,light);
    c_max = light_phong(mesh.point[m_max],mesh.color,mesh.point_normal[m_max],eye,light);
    //第一次扫描转换:从y轴最小点扫描转换至分界点
    d0 = (mesh.point[m_min].x-mesh.point[m_mid].x)/(mesh.point[m_min].y-mesh.point[m_mid].y); //计算第一条直线的1/k
    d1 = (mesh.point[m_min].x-mesh.point[m_max].x)/(mesh.point[m_min].y-mesh.point[m_max].y); //计算第二条直线的1/k
    nd0 = (c_min-c_mid)/(mesh.point[m_min].y-mesh.point[m_mid].y);
    nd1 = (c_min-c_max)/(mesh.point[m_min].y-mesh.point[m_max].y);
    x0 = x1 = mesh.point[m_min].x; //将两条直线的x置为起点(y最小点)的x
    n0 = n1 = c_min;
    if((int)mesh.point[m_min].y<=0) //若起点位于屏幕外进行裁切
    {
        x0 += d0*-mesh.point[m_min].y;
        x1 += d1*-mesh.point[m_min].y;
        n0 += nd0*-mesh.point[m_min].y;
        n1 += nd1*-mesh.point[m_min].y;
    }
    for(int i=max((int)mesh.point[m_min].y,0);i<=min((int)mesh.point[m_mid].y,HEIGHT);i++) //对于面元在屏幕内经过的每一个y
    {
        int s_min=max(x_min,max((int)min(x0,x1),0)),s_max=min(x_max,min((int)max(x0,x1),WIDTH));
        if(x0<x1) {n=n0;nd=(n1-n0)/(s_max-s_min);}
        else {n=n1;nd=(n0-n1)/(s_max-s_min);}
        for(int j=s_min;j<=s_max;j++) //对于扫描线在当前y和图形与屏幕区域相交的每一个x
        {
            float z=get_Z(mesh,j,i);
            put_point(j,i,(int)z,color_vector2int(n)); //调用绘制点函数绘制当前x,y对应点,其中z坐标由get_Z函数获取,颜色即为当前面元颜色
            n+=nd;
        }
        x0 += d0; //更新第一条直线与扫描线交点的x值
        x1 += d1; //更新第二条直线与扫描线交点的x值
        n0 += nd0;
        n1 += nd1;
    }
    //第二次扫描转换:从y轴最大点扫描转换至分界点(代码相同,不进行重复注释)
    d0 = (mesh.point[m_max].x-mesh.point[m_mid].x)/(mesh.point[m_max].y-mesh.point[m_mid].y);
    d1 = (mesh.point[m_max].x-mesh.point[m_min].x)/(mesh.point[m_max].y-mesh.point[m_min].y);
    nd0 = (c_max-c_mid)/(mesh.point[m_max].y-mesh.point[m_mid].y);
    nd1 = (c_max-c_min)/(mesh.point[m_max].y-mesh.point[m_min].y);
    x0 = x1 = mesh.point[m_max].x;
    n0 = n1 = c_max;
    if((int)mesh.point[m_max].y>=HEIGHT)
    {
        x0 -= d0*(mesh.point[m_max].y-HEIGHT);
        x1 -= d1*(mesh.point[m_max].y-HEIGHT);
        n0 += nd0*(mesh.point[m_max].y-HEIGHT);
        n1 += nd1*(mesh.point[m_max].y-HEIGHT);
    }
    for(int i=min((int)mesh.point[m_max].y,HEIGHT);i>=max((int)mesh.point[m_mid].y,0);i--)
    {
        int s_min=max(x_min,max((int)min(x0,x1),0)),s_max=min(x_max,min((int)max(x0,x1),WIDTH));
        if(x0<x1) {n=n0;nd=(n1-n0)/(s_max-s_min);}
        else {n=n1;nd=(n0-n1)/(s_max-s_min);}
        for(int j=s_min;j<=s_max;j++) //对于扫描线在当前y和图形与屏幕区域相交的每一个x
        {
            float z=get_Z(mesh,j,i);
            put_point(j,i,(int)z,color_vector2int(n)); //调用绘制点函数绘制当前x,y对应点,其中z坐标由get_Z函数获取,颜色即为当前面元颜色
            n+=nd;
        }
        x0 -= d0; //更新第一条直线与扫描线交点的x值
        x1 -= d1; //更新第二条直线与扫描线交点的x值
        n0 -= nd0;
        n1 -= nd1;
    }
    return 0;
}

//三维模型扫描转换函数
int scan_batch(vector<Mesh> model,vector<Light> light,Vector4 eye)
{
    //对模型中的每个面元都调用scan函数进行扫描转换
    for(vector<Mesh>::iterator i=model.begin();i!=model.end();i++)
        scan(*i,light,eye);
    return 0;
}

//将帧缓冲器的内容刷新至显示窗口
int flush()
{
    glBegin(GL_POINTS);
    //遍历帧缓冲器的每一项
    for (int i = 0; i < HEIGHT; i++) {
        for (int j = 0; j < WIDTH; j++) {
            //将颜色置为该项内存储的值,其中[7:0]为蓝色,[15:8]为绿色,[23:16]为红色
            glColor3ub((FrameBuffer[i][j] >> 16) & 0x000000FF,
                (FrameBuffer[i][j] >> 8) & 0x000000FF,
                (FrameBuffer[i][j] >> 0) & 0x000000FF);
            //在当前坐标调用OpenGL绘制点
            glVertex2i(j, i);
        }
    }
    glEnd();
    glFlush();
    return 0;
}

//实现2PI范围内的反正切函数
float get_degree(float dy, float dx)
{
    if (dx >= 0) return atan(dy / dx);
    else return atan(dy / dx) + PI;
}

//视角变换矩阵生成函数
Matrix4 get_eye_transform_matrix(Vector4 position, Vector4 direction, Vector4 upper)
{
    if (abs(direction.z) < EPSILON) //避免视线方向完全处于XY平面导致无法按预置的旋转规则旋转至指定方向
        direction.z = EPSILON; //将视线方向的Z值置为微小量
    Matrix4 T1, T2, T3, T4, T5;
    float axis_rotate1, axis_rotate2, axis_rotate3;
    //需要两次基本旋转变换将任意方向的视线方向转至与Z轴重合
    axis_rotate1 = get_degree(direction.y, direction.z); //第一次旋转的角度
    axis_rotate2 = get_degree(direction.x, direction.z / cos(axis_rotate1)); //第二次旋转的角度
    //将视点平移至原点的变换矩阵
    T1.set(1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        -position.x, -position.y, -position.z, 1);
    //第一次基本旋转变换矩阵
    T2.set(1, 0, 0, 0,
        0, cos(axis_rotate1), sin(axis_rotate1), 0,
        0, -sin(axis_rotate1), cos(axis_rotate1), 0,
        0, 0, 0, 1);
    //第二次基本旋转变换矩阵
    T3.set(cos(axis_rotate2), 0, -sin(axis_rotate2), 0,
        0, 1, 0, 0,
        sin(axis_rotate2), 0, cos(axis_rotate2), 0,
        0, 0, 0, 1);
    upper = T3 * T2 * T1 * upper; //拿到变换后向上方向的向量
    axis_rotate3 = get_degree(upper.x, upper.y); //通过向上向量计算出需要在XOY二维平面上的旋转角度
    //矫正向上方向旋转变换矩阵
    T4.set(cos(axis_rotate3), sin(axis_rotate3), 0, 0,
        -sin(axis_rotate3), cos(axis_rotate3), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);
    //从投影完成的世界坐标系(在XOY面上)转换至屏幕坐标系的变换矩阵
    T5.set(1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, 1, 0,
        WIDTH >> 1, HEIGHT >> 1, 0, 1);
    return T5 * T4 * T3 * T2 * T1; //将各矩阵相乘拿到复合变换矩阵并返回
}

//三角面元三维变换函数
Mesh transform(Mesh mesh,Matrix4 T)
{
    //对于面元的每个点,将其与变换矩阵相乘并更新坐标值
    for(int i=0;i<3;i++)
    {
        Vector4 buf;
        buf=T*mesh.point[i];
        mesh.point[i]=buf;
        buf=T*mesh.point_normal[i];
        mesh.point_normal[i]=buf;
    }
    get_vector(mesh); //重新计算面元向量
    return mesh;
}

//三维模型三维变换函数
vector<Mesh> transform_batch(vector<Mesh> model,Matrix4 T)
{
    vector<Mesh> res;
    //对于多边形的每一个面元,调用transform函数拿到变换后的面元,推入新多边形
    for(vector<Mesh>::iterator i=model.begin();i!=model.end();i++)
        res.push_back(transform(*i,T));
    return res;
}

vector<Light> transform_batch(vector<Light> light,Matrix4 T)
{
    vector<Light> res;
    for(vector<Light>::iterator i=light.begin();i!=light.end();i++)
        res.push_back(Light{T*i->position,i->color});
    return res;
}


//OBJ文件加载函数
vector<Mesh> obj_loader(const string& path)
{
    vector<Mesh> res;
    ifstream fs;
    fs.open(path);
    if (!fs.is_open())
    {
        cout << "OBJ File Not Found: " << path << endl;
        return res;
    }
    vector<Vector4> point,point_normal;
    vector<Vector2> mipmap;
    string header;
    while (!fs.eof())
    {
        fs >> header; //从文件输入流读入行首
        if (header == "v")
        {
            //若是顶点,将其推入顶点表
            float x, y, z;
            fs >> x >> y >> z;
            point.push_back({ x,y,z,1 });
        }
        else if (header == "vn")
        {
            float x, y, z;
            fs >> x >> y >> z;
            point_normal.push_back(Vector4{ x,y,z,0 }.normalize());
        }
        else if (header == "vt")
        {
            float x, y;
            fs >> x >> y;
            mipmap.push_back({ x, y });
        }
        else if (header == "f") //若是面
        {
            string buf;
            getline(fs,buf);
            istringstream is(buf);
            vector<int> point_index;
            vector<int> point_normal_index;
            while(!is.eof())
            {
                string s;
                int v_index,vt_index,vn_index;
                char c;
                is>>s;
                istringstream is2(s);
                is2 >> v_index >> c >> vt_index >> c >> vn_index;
                point_index.push_back(v_index); //通过多次输入流,最终拿到每个顶点的整形下标,推入顶点下标表
                point_normal_index.push_back(vn_index);
            }
            //int color=rand(); //每个面元"随机"选一个颜色
            int color=0xFFFFFF;
            //如果顶点数大于等于三个,将其拆分为若干个三角面元并推入面元向量,其中顶点坐标由下标访问之前得到的顶点表拿到
            if(point_index.size()>=3)
                for(int i=1;i<point_index.size()-1;i++)
                    res.push_back({{ point[point_index[0]-1],point[point_index[i]-1],point[point_index[i+1]-1]},
                                   {point_normal[point_normal_index[0]-1],point_normal[point_normal_index[i]-1],point_normal[point_normal_index[i+1]-1]},
                                   {0,0,0,0},{{0,0},{0,0},{0,0}},color});
        }
    }
    cout << "OBJ File Open Success: " << path << endl;
    return res;
}

vector<vector<Vector3>> mipmap_loader(const string& path)
{
    vector<vector<Vector3>> res;
    vector<unsigned char> m_data;
    ifstream fs(path, ios_base::in | ios_base::beg | ios_base::binary);
    if (!fs.is_open())
    {
        printf("open BMP failed!\n");
    }
    else
    {
        printf("open BMP successed!\n");
    }
    int offbits;    // BMP读到像素位置的偏移量
    int biWidth;    // 图片宽、高
    int biHeight;
    fs.seekg(10, ios_base::beg);
    fs.read((char*)(&offbits), sizeof(int));
    printf("offbits is : %d\n", offbits);
    fs.read((char*)(&biWidth), sizeof(int));
    fs.read((char*)(&biHeight), sizeof(int));
    printf("W,H is : %d  %d\n", biWidth, biHeight);

    fs.seekg(offbits, ios_base::beg);
    unsigned char data;
    for (int i = 0; i < biHeight; i++)
        for (int j = 0; j < (biWidth * 3); j++)
        {
            fs.read((char*)(&data), sizeof(char));
            m_data.push_back(data);
        }
    for (int i = biHeight - 1; i >= 0; i--)    // 将图片像素上下翻转打包成二维向量
    {
        vector<Vector3> pixels;
        for (int j = (i * biWidth + biWidth * 3 - 3); j >= (i * biWidth); j -= 3)
        {
            pixels.push_back({ (float)m_data[j],(float)m_data[j + 1],(float)m_data[j + 2] });
        }
        res.push_back(pixels);
    }
    for (int i = 0; i < biHeight; i++)
    {
        for (int j = 0; j < biWidth; j++)
        {
            if (res[i][j][0] == 255)
                printf(". ");
            else
            {
                printf("# ");
            }
        }
        printf("\n");
    }
    return res;
}














//以下为实例使用部分
float degree = 0;
float degree_z = -0.3;
vector<Mesh> model;
vector<Light> light;

//自带模型绘制函数
void draw_model()
{
    clear(0xFFFFFF);
    Matrix4 T=get_eye_transform_matrix({0,0,0,1},{sin(degree),cos(degree),degree_z,0},{0,0,1,0});
    scan_batch(transform_batch(model,T), transform_batch(light,T),{0,0,0,1});
    glutPostRedisplay();
}

void keyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
    case '1': //按1初始化并绘制自带模型
    {
        vector<vector<Vector3>> bitmap = mipmap_loader(MAP_PATH); // unsupported yet
        break;
    }
    case ' ': //按空格初始化,从OBJ文件加载三维模型并进行一个初始三维变换(实现以合适大小显示在屏幕中央的效果)后显示
    {
        light.push_back({{-10000,10000,10000,1},0xFF0000});
        light.push_back({{-10000,-10000,10000,1},0x0000FF});
        model = obj_loader(OBJ_PATH);
        Matrix4 T;
        //本基本变换矩阵适用于lily-impeller.obj
        T.set(3, 0, 0, 0,
            0, 3, 0, 0,
            0, 0, 3, 0,
            0, 0, -600, 1);
        model = transform_batch(model, T);
        draw_model();
        break;
    }
    //ZX键实现视角横向旋转
    case 'z':
    {
        degree += 0.2;
        draw_model();
        break;
    }
    case 'x':
    {
        degree -= 0.2;
        draw_model();
        break;
    }
    //CV键实现视角纵向旋转
    case 'c':
    {
        degree_z += 0.1;
        draw_model();
        break;
    }
    case 'v':
    {
        degree_z -= 0.1;
        draw_model();
        break;
    }
    //BN键缩放三维模型
    case 'b':
    {
        Matrix4 T;
        T.set(1.1, 0, 0, 0,
            0, 1.1, 0, 0,
            0, 0, 1.1, 0,
            0, 0, 0, 1);
        model = transform_batch(model, T);
        draw_model();
        break;
    }
    case 'n':
    {
        Matrix4 T;
        T.set(0.9, 0, 0, 0,
            0, 0.9, 0, 0,
            0, 0, 0.9, 0,
            0, 0, 0, 1);
        model = transform_batch(model, T);
        draw_model();
        break;
    }
    //O键打印三维模型所有顶点坐标
    case 'o':
    {
        cout << model.size() << endl;
        for (vector<Mesh>::iterator i = model.begin(); i != model.end(); i++)
            cout << i->point[0] << i->point[1] << i->point[2] << endl;
        break;
    }
    case 27:
        exit(0);
    }
}
void display(void)
{
    flush();
}
void reshape(int w, int h)
{
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);//视口大小
    glMatrixMode(GL_PROJECTION);//设置投影模式以及视景体大小
    glLoadIdentity();
    glOrtho(0, (GLsizei)w, (GLsizei)h, 0, -ZVALUE, ZVALUE);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void mouseClick(int button, int state, int x, int y)
{
    int centerX = HEIGHT / 2;
    int centerY = WIDTH / 2;

    if (button == GLUT_LEFT_BUTTON)
    {
        if (state == GLUT_DOWN)
        {
            float deltaX = x - centerX;
            float deltaY = y - centerY;
            degree += deltaX * 0.5 / WIDTH;
            degree_z -= deltaY * 0.25 / HEIGHT;
            draw_model();
        }
    }
}


int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
    glutInitWindowSize(WIDTH, HEIGHT);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("ZBuffer Renderer");

    glutReshapeFunc(reshape);
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouseClick);
    glutMainLoop();
    return 0;
}
