#include <iostream>
#include <svpng.inc>
#include <glm/glm.hpp>
#include <cstdio>
#include <vector>
#include <random>
#include <ctime>
using namespace glm;
using namespace std;

// 输出图像分辨率
const int WIDTH = 256;
const int HEIGHT = 256;

// 采样次数
const int SAMPLE = 4096;
// 每次采样的亮度
const double BRIGHTNESS = (2.0f * 3.1415926f) * (1.0f / double(SAMPLE));


// 相机参数
const double SCREEN_Z = 1.1;        // 视平面 z 坐标
const vec3 EYE = vec3(0, 0, 4.0);   // 相机位置

// 颜色
const vec3 RED(1, 0.5, 0.5);
const vec3 GREEN(0.5, 1, 0.5);
const vec3 BLUE(0.5, 0.5, 1);
const vec3 YELLOW(1.0, 1.0, 0.1);
const vec3 CYAN(0.1, 1.0, 1.0);
const vec3 MAGENTA(1.0, 0.1, 1.0);
const vec3 GRAY(0.5, 0.5, 0.5);
const vec3 WHITE(1, 1, 1);

typedef struct Ray{
    vec3 startPoint = vec3(0, 0, 0);
    vec3 direction  = vec3(0, 0, 0);
}Ray;

typedef struct Material{
    bool isEmissive     = false;                    // 是否发光
    vec3 normal         = vec3(0, 0, 0);    // 法向量
    vec3 color          = vec3(0, 0, 0);    // 颜色
    double specularRate = 0.0f;                     // 反射率, 反射光占比
    double roughness    = 1.0f;                     // 粗糙程度
    double refractRate  = 0.0f;                     // 折射光占比
    double refractAngle = 1.0f;                     // 折射率
    double refractRoughness = 0.0f;                 // 折射粗糙度

}Material;

typedef struct HitResult{
    bool isHit      = false;                    // 是否命中
    double distance = 0.0f;                     // 与交点的距离
    vec3 hitPoint   = vec3(0, 0, 0);    // 光线命中点
    Material        material;                   // 命中点表面的材质
};

class Shape{
public:
    Shape(){}
    virtual HitResult intersect(Ray ray){   return HitResult(); }
};

class Triangle : public Shape{
public:
    Triangle(){}
    Triangle(vec3 P1, vec3 P2, vec3 P3, vec3 C){
        p1 = P1;
        p2 = P2;
        p3 = P3;
        material.normal = normalize(cross(p2-p1, p3-p1));
        material.color = C;
    }
    HitResult intersect(Ray ray){
        HitResult res;

        vec3 S = ray.startPoint;
        vec3 d = ray.direction;
        vec3 N = material.normal;
        if(dot(N, d) > 0)   N = -N;     // 获取正确的法向量

        // 如果视线和三角形平行
        if(fabs(dot(N, d)) < 0.00001f)  return res;

        // 距离
        float t = (dot(N, p1) - dot(S, N)) / dot(d, N);
        if(t < 0.00005f)    return res;     // 如果三角形在相机背面

        // 交点计算
        vec3 P = S + d * t;

        // 判断交点是否在三角形内
        vec3 c1 = cross(p2-p1, P-p1);
        vec3 c2 = cross(p3-p2, P-p2);
        vec3 c3 = cross(p1-p3, P-p3);
        vec3 n  = material.normal;  // 需要 "原生法向量" 来判断
        if(dot(c1, n)<0 || dot(c2, n)<0 || dot(c3,n)<0)     return res;

        // 装填返回结果
        res.isHit           = true;
        res.distance        = t;
        res.material        = material;
        res.hitPoint        = P;
        res.material.normal = N;

        return res;
    }
public:
    vec3 p1, p2, p3;    // 顶点
    Material material;  // 材质
};

class Sphere : public Shape{
public:
    Sphere(){};
    Sphere(vec3 o, double r, vec3 c, double spec, double rough){
        O = o;
        R = r;
        material.color = c;
        material.specularRate = spec;
        material.roughness = rough;
    }

    HitResult intersect(Ray ray){
        HitResult res;
        vec3 S = ray.startPoint;
        vec3 Dir = ray.direction;
        float OS = length(O-S);
        float SH = dot(O-S, Dir);
        float OH = sqrt(pow(OS, 2) - pow(SH, 2));
        float PH = sqrt(pow(R, 2) - pow(OH, 2));

        if(OH > R)      return res;     // OH大于半径则不相交
        float t1 = length(SH) - PH;
        float t2 = length(SH) + PH;
        float t = (t1<0) ? t2 : t1;     // 最近距离
        vec3 P = S + t*Dir;             // 交点

        // 防止自己交自己
        if(fabs(t1)<0.0005f || fabs(t2)<0.0005f)    return res;

        // 装填返回结果
        res.isHit = true;
        res.material = material;
        res.hitPoint = P;
        res.distance = t;
        res.material.normal = normalize(P - O); // 要返回正确的法向
        return res;
    }

public:
    vec3 O;
    double R;
    Material material;
};

vector<Shape*> shapes;      // 几何物体的集合

void imshow(double* SRC);
HitResult shoot(vector<Shape*> shapes, Ray ray);
vec3 randomVec3();
vec3 randomDirection(vec3 n);
vec3 pathTracing(vector<Shape*> shapes, Ray ray, int depth);
tm getTime();
double randf();

std::uniform_real_distribution<> dis(0.0, 1.0);
random_device rd;
mt19937 gen(rd());

int main() {
    double* image = new double[WIDTH * HEIGHT * 3];
    memset(image, 0.0, sizeof(double) * WIDTH * HEIGHT * 3);
    auto startTime = getTime();
    std::cerr << "START_TIME:" << startTime.tm_hour << ':' << startTime.tm_min << ':' << startTime.tm_sec << std::endl;


    // // 三角形
    // shapes.push_back(new Triangle(vec3(-0.5, -0.5, -0.5), vec3(0.5, -0.5, -0.5), vec3(0, -0.5, 0.5), CYAN));
    // 底部平面
    shapes.push_back(new Triangle(vec3(1, -1, 1), vec3(-1, -1, -1), vec3(-1, -1, 1), WHITE));
    shapes.push_back(new Triangle(vec3(1, -1, 1), vec3(1, -1, -1), vec3(-1, -1, -1), WHITE));
    // 右部平面
    shapes.push_back(new Triangle(vec3(1, 1, 1), vec3(1, -1, -1), vec3(1, -1, 1), RED));
    shapes.push_back(new Triangle(vec3(1, -1, -1), vec3(1, 1, 1), vec3(1, 1, -1), RED));
    // 左部平面
    shapes.push_back(new Triangle(vec3(-1, -1, -1), vec3(-1, 1, 1), vec3(-1, -1, 1), BLUE));
    shapes.push_back(new Triangle(vec3(-1, -1, -1), vec3(-1, 1, -1), vec3(-1, 1, 1), BLUE));
    // 上部平面
    shapes.push_back(new Triangle(vec3(1, 1, 1), vec3(-1, 1, 1), vec3(-1, 1, -1), WHITE));
    shapes.push_back(new Triangle(vec3(1, 1, 1), vec3(-1, 1, -1), vec3(1, 1, -1), WHITE));
    // 后部平面
    shapes.push_back(new Triangle(vec3(1, -1, -1), vec3(-1, 1, -1), vec3(-1, -1, -1), CYAN));
    shapes.push_back(new Triangle(vec3(1, -1, -1), vec3(1, 1, -1), vec3(-1, 1, -1), CYAN));

    // 球
    // shapes.push_back(new Sphere(vec3(-0.6, -0.8, 0.6), 0.2, WHITE, 0.9, 0.2));
    shapes.push_back(new Sphere(vec3(-0.1, -0.7, 0.2), 0.3, WHITE, 0.9, 0.6));
    shapes.push_back(new Sphere(vec3(0.5, -0.6, -0.5), 0.4, WHITE, 0.8, 0.8));
    Sphere s1 = Sphere(vec3(-0.6, -0.8, 0.6), 0.2, WHITE, 0.2, 0.2);
    s1.material.refractRoughness = 0.1;
    s1.material.refractRate = 0.8;
    shapes.push_back(&s1);

    // 光源
    Triangle l1 = Triangle(vec3(0.4, 0.99, 0.4), vec3(-0.4, 0.99, -0.4), vec3(-0.4, 0.99, 0.4), WHITE);
    Triangle l2 = Triangle(vec3(0.4, 0.99, 0.4), vec3(0.4, 0.99, -0.4), vec3(-0.4, 0.99, -0.4), WHITE);
    l1.material.isEmissive = true;
    l2.material.isEmissive = true;
    shapes.push_back(&l1);
    shapes.push_back(&l2);

    for(int k=0;k<SAMPLE;k++){
        double* p = image;
        double percentage = (double)k/(double)SAMPLE;
        std::cerr << "\rWORKING: " << percentage*100 << '%' << std::flush;

        for (int i = 0; i < HEIGHT; i++){
            for (int j = 0; j < WIDTH; j++){
                // 像素坐标转投影平面坐标
                double x = 2.0 * double(j) / double(WIDTH) - 1.0;
                double y = 2.0 * double(HEIGHT - i) / double(HEIGHT) - 1.0;

                // MSAA
                x += (randf() - 0.5f) / (double)WIDTH;
                y += (randf() - 0.5f) / (double)HEIGHT;

                vec3 coord = vec3(x, y, SCREEN_Z);          // 计算投影平面坐标
                vec3 direction = normalize(coord - EYE);    // 计算光线投射方向

                // 生成光线
                Ray ray;
                ray.startPoint  = coord;
                ray.direction   = direction;

                // 找交点并输出交点的颜色
                HitResult res   = shoot(shapes, ray);
                // vec3 color      = vec3(0, 0, 0);
                vec3 color      = res.material.color;
                if(res.isHit){
                    // 命中光源直接返回光源颜色
                    if(res.material.isEmissive){
                        color = res.material.color;
                    }
                    else{
                        // 根据交点处法向量生成交点处反射的随机半球向量
                        Ray randomRay;
                        randomRay.startPoint = res.hitPoint;
                        randomRay.direction  = randomDirection(res.material.normal);

                        float r = randf();
                        if (r < res.material.specularRate){  // 镜面反射

                            vec3 reflect = normalize(glm::reflect(ray.direction, res.material.normal));
                            randomRay.direction = mix(reflect, randomRay.direction, res.material.roughness);
                            color = pathTracing(shapes, randomRay, 0);
                        }
                        else if(r < res.material.refractRate && r > res.material.specularRate){
                            vec3 refract = normalize(glm::refract(ray.direction, res.material.normal, (float)res.material.refractAngle));
                            randomRay.direction = mix(refract, -randomRay.direction, res.material.refractRoughness);
                            color = pathTracing(shapes, randomRay, 0);
                        }
                        else{
                            // 颜色积累
                            vec3 srcColor   = res.material.color;
                            vec3 ptColor    = pathTracing(shapes, randomRay, 0);
                            color           = ptColor * srcColor;
                        }
                        color *= BRIGHTNESS;
                    }
                }

                *p += color.x; p++;  // R 通道
                *p += color.y; p++;  // G 通道
                *p += color.z; p++;  // B 通道
            }
        }
    }
    auto overTime = getTime();
    int cost_hour = overTime.tm_hour - startTime.tm_hour;
    if(cost_hour < 0)   cost_hour += 24;
    int cost_min = overTime.tm_min - startTime.tm_min;
    if(cost_min < 0){
        cost_hour--;
        cost_min += 60;
    }
    int cost_sec = overTime.tm_sec - startTime.tm_sec;
    if(cost_sec < 0){
        cost_min--;
        cost_sec += 60;
    }
    std::cerr << "\rWORKING: 100%" << std::flush;
    std::cerr << "\nDONE." << "\nOVER_TIME:" << overTime.tm_hour << ':' << overTime.tm_min << ':' << overTime.tm_sec
                << "\nCOST:" << cost_hour << ':' << cost_min << ':' << cost_sec;
    imshow(image);
    return 0;
}

void imshow(double* SRC){
    unsigned char* image = new unsigned char[WIDTH * HEIGHT * 3];   // 图像buffer
    unsigned char* p = image;
    double* S = SRC;    // 源数据
    FILE* fp = fopen("../image_8_Final.png", "wb");
    for(int i=0;i<HEIGHT;i++){
        for(int j=0;j<WIDTH;j++){
            *p++ = (unsigned char)clamp(pow(*S++, 1.0f / 2.2f) * 255, 0.0, 255.0);  // R 通道
            *p++ = (unsigned char)clamp(pow(*S++, 1.0f / 2.2f) * 255, 0.0, 255.0);  // G 通道
            *p++ = (unsigned char)clamp(pow(*S++, 1.0f / 2.2f) * 255, 0.0, 255.0);  // B 通道
        }
    }
    svpng(fp, WIDTH, HEIGHT, image, 0);
}

HitResult shoot(vector<Shape*> shapes, Ray ray){
    HitResult res, r;
    res.distance = INFINITY;
    for(auto& shape : shapes){
        r = shape->intersect(ray);
        if(r.isHit && r.distance < res.distance)    res = r;    // 记录距离最近的求交结果
    }
    return res;
}

vec3 randomVec3(){
    vec3 d;
    do{
        d = 2.0f * vec3(randf(), randf(), randf()) - vec3(1.0, 1.0, 1.0);
    }while(dot(d, d) > 1.0f);
    return normalize(d);
}

vec3 randomDirection(vec3 n){
    return normalize(randomVec3() + n);
}

vec3 pathTracing(vector<Shape*> shapes, Ray ray, int depth){
    // 大于8层直接返回
    if(depth > 8)   return vec3(0, 0, 0);

    HitResult res = shoot(shapes, ray);

    // 未命中
    if(!res.isHit)                  return vec3(0, 0, 0);

    // 如果发光则返回颜色
    if(res.material.isEmissive)     return res.material.color;

    // 有 P 的概率终止
    double r = randf();
    float P = 0.8;
    if(r > P)   return vec3(0, 0, 0);

    // 否则继续
    Ray randomRay;
    randomRay.startPoint = res.hitPoint;
    randomRay.direction  = randomDirection(res.material.normal);

    vec3 color = vec3(0, 0, 0);
    float cosine = dot(-ray.direction, res.material.normal);

    // 根据反射率决定光线最终的方向
    r = randf();
    // 当随机数小于 reflectRate 的时候发生反射，
    // 随机数在 reflectRate 和 refractRate 之间发生折射，
    // 随机数大于 refractRate 的时候才是漫反射
    if(r < res.material.specularRate){      // 镜面反射
        // 我们反射的时候不再按照反射光线的方向，而是根据粗糙度，在随机向量和反射光线的方向做一个*线性插值*以决定最终反射的方向
        vec3 reflect = normalize(glm::reflect(ray.direction ,res.material.normal));
        randomRay.direction = mix(reflect, randomRay.direction, res.material.roughness);
        color = pathTracing(shapes, randomRay, depth+1) * cosine;
    }
    else if(r<res.material.refractRate && r>res.material.specularRate){
        vec3 refract = normalize(glm::refract(ray.direction, res.material.normal, float(res.material.refractAngle)));
        randomRay.direction = mix(refract, -randomRay.direction, res.material.refractRoughness);
        color = pathTracing(shapes, randomRay, depth+1) * cosine;
    }
    else{
        vec3 srcColor = res.material.color;
        vec3 ptColor = pathTracing(shapes, randomRay, depth+1) * cosine;
        color = ptColor * srcColor;    // 和原颜色混合
    }

    // 否则直接返回
    return color/P;
}

tm getTime(){
    time_t rawtime;
    struct tm *ptminfo;
    time(&rawtime);
    ptminfo = localtime(&rawtime);
    return *ptminfo;
}

double randf(){
    return dis(gen);
}