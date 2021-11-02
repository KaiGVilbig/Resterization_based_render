#include <vector>
#include <Eigen/Dense>

class Polygon
{
    public:
        void setSize(int);
        int getSize();

        void setCoords(std::vector<float>);
        std::vector<Eigen::Vector3d> getCoords();

        void setNorms(std::vector<float>);
        std::vector<Eigen::Vector3d> getNorms();

        void setColor(std::vector<float>);
        float getColor(int);

        void setType(std::string);
        std::string getType();

        void setscoords(std::vector<float>);
        std::vector<float> getscoords();

        void setVColor(std::vector<float>);
        float getVColor(int);

        void setMCoords(Eigen::Vector4d c);
        std::vector<Eigen::Vector4d> getMCoords();

        void setDel(bool t);
        bool getDel();

    private:
        bool del = false;
        int polySize;
        std::vector<Eigen::Vector3d> coords;
        std::vector<Eigen::Vector3d> norms;
        std::vector<float> scoords;
        std::vector<float> color;
        std::string type;
        std::vector<float> vcolor;
        std::vector<Eigen::Vector4d> mCoords;
};

class HitRecord
{
    public:
        void setZ(double);
        double getZ();

        void setColor(Eigen::Vector3d);
        Eigen::Vector3d getColor();

    private:
        Eigen::Vector3d color;
        double zBuffer;
};

class Camera
{
    public:
        Eigen::Vector3d getW();
        void setW(Eigen::Vector3d);

        Eigen::Vector3d getU();
        void setU(Eigen::Vector3d);

        Eigen::Vector3d getV();
        void setV(Eigen::Vector3d);

    private:
        Eigen::Vector3d w;
        Eigen::Vector3d u;
        Eigen::Vector3d v;
};