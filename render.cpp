#include <iostream>
#include <math.h>

#include "render.h"

double max(double a, double b)
{
    if (a > b)
    {
        return a;
    }
    return b;
}

void splitString(std::string toSplit, char delim, std::vector<std::string> *storeSplit)
{
        std::stringstream tmpData;
        std::string tmp;
        tmpData << toSplit;
        while (std::getline(tmpData, tmp, delim)) 
        {
            storeSplit->push_back(tmp);
        }
}

void hrInit(std::vector<std::vector<HitRecord> > *hr, int x, int y, Eigen::Vector3d bg)
{
    for (int i = 0; i < y; i++)
    {
        std::vector<HitRecord> r;
        for (int j = 0; j < x; j++)
        {
            HitRecord h;
            h.setColor(bg);
            h.setZ(-1000);
            r.push_back(h);
        }
        hr->push_back(r);
    }
}   

void matrixInit(Eigen::MatrixXd *Mvp, Eigen::MatrixXd *Morth, Eigen::MatrixXd *Mcam, Eigen::MatrixXd *P, Camera cam, Eigen::Vector3d origin,
                double near, double far, double right, double left, double top, double bottom, double width, double height)
{
    Eigen::MatrixXd P1(4, 4);
    P1 <<   -cam.getU()(0), -cam.getU()(1), -cam.getU()(2), 0,
            -cam.getV()(0), -cam.getV()(1), -cam.getV()(2), 0,
            -cam.getW()(0), -cam.getW()(1), -cam.getW()(2), 0,
            0, 0, 0, 1;
    Eigen::MatrixXd P2(4, 4);
    P2 <<   1, 0, 0, -origin(0),
            0, 1, 0, -origin(1),
            0, 0, 1, -origin(2),
            0, 0, 0, 1;


    *Mvp << (width / 2),    0,              0,      ((width - 1) / 2),
            0,              (height / 2),   0,      ((height - 1) / 2),
            0,              0,              1,      0,
            0,              0,              0,      1;

    *Morth << (2 / (right - left)),     0,                      0,                      -((right + left) / (right - left)),
              0,                        (2 / (top - bottom)),   0,                      -((top + bottom) / (top - bottom)),
              0,                        0,                      (2 / (near - far)),     -((near + far) / (near - far)),
              0,                        0,                      0,                      1;

    *Mcam = P1 * P2;

    *P <<   near,       0,          0,              0,
            0,          near,       0,              0,
            0,          0,          (near + far),   -(far * near),
            0,          0,          1,              0;
}

// Open the nff file and parse the data
int parseNff(std::string image, Eigen::Vector3d *background, std::vector<Eigen::Vector3d> *viewpoint, std::vector<Polygon> *polygon, std::vector<Eigen::Vector3d> *light)
{
    std::fstream nff_file;
    nff_file.open(image, std::ios::in);

    if (!nff_file.is_open())
    {
        std::cout << "File unable to be opened\n";
    }
    else
    {
        std::string data;

        // Get background info
        std::getline(nff_file, data);
        std::vector<std::string> backgroundData;
        splitString(data, ' ', &backgroundData);
        Eigen::Vector3d b(stof(backgroundData[1]), stof(backgroundData[2]), stof(backgroundData[3]));
        *background = b;

        // Get viewpoint info
        std::getline(nff_file, data);
        // next 6 getlines should correspond to from, at, up, angle, hither and resolution.
        for (int i = 0; i < 6; i++)
        {
            // Get info
            std::getline(nff_file, data);
            std::vector<std::string> viewpointData;
            splitString(data, ' ', &viewpointData);

            // first 3 have all three numbers, angle and hither only have 1 and res has 2
            if (i < 3) 
            {
                Eigen::Vector3d v(stod(viewpointData[1]), stod(viewpointData[2]), stod(viewpointData[3]));
                viewpoint->push_back(v);
            }
            else if (i < 5) 
            {
                Eigen::Vector3d v(stof(viewpointData[1]), 0, 0);
                viewpoint->push_back(v);
            }
            else
            {
                Eigen::Vector3d v(stof(viewpointData[1]), stof(viewpointData[2]), 0);
                viewpoint->push_back(v);
            }
        }

        std::vector<float> colorData;

        // Get polygons
        while (std::getline(nff_file, data))
        {
            // Get fill color info
            std::vector<std::string> fillData;
            splitString(data, ' ', &fillData);

            while (fillData[0] == LIGHT)
            {
                Eigen::Vector3d l(stof(fillData[1]), stof(fillData[2]), stof(fillData[3]));
                light->push_back(l);
                fillData.clear();
                std::getline(nff_file, data);
                splitString(data, ' ', &fillData);
            }

            Polygon p;
            
            // check if next data is polygon or color info
            if (fillData[0] == FILL_COLOR)
            {
                colorData.clear();
                for (unsigned int i = 1; i < fillData.size(); i++) 
                {
                    colorData.push_back(stof(fillData[i]));
                }  
                fillData.clear();
                std::getline(nff_file, data);
                splitString(data, ' ', &fillData);
            }
            p.setColor(colorData);

            // Set size of polygon
            if (fillData[0] == POLYGON || fillData[0] == POLY_PATCH)
            {
                p.setType(fillData[0]);
                int loopNum = stoi(fillData[1]);
                p.setSize(3);

                // Set coords
                std::vector<float> fcoordsData;
                std::vector<float> normData;
                for (int i = 0; i < loopNum; i++)
                {
                    std::getline(nff_file, data);
                    std::vector<std::string> coordsData;
                    splitString(data, ' ', &coordsData);
                    // Set coords to float

                    for (unsigned int j = 0; j < coordsData.size(); j++)
                    {
                        if (fillData[0] == POLYGON || j < (coordsData.size() / 2))
                        {
                            fcoordsData.push_back(stof(coordsData[j]));
                        }
                        else
                        {
                            normData.push_back(stof(coordsData[j]));
                        }
                    }
                    // Support for polygons > 3 sides
                    if (i >= 2)
                    {
                        if (i > 2)
                        {
                            Polygon po;
                            po.setColor(colorData);
                            po.setSize(3);
                            po.setCoords(fcoordsData);
                            po.setNorms(normData);
                            po.setType(fillData[0]);
                            polygon->push_back(po);
                        }
                        else
                        {
                            p.setNorms(normData);
                            p.setCoords(fcoordsData);
                            // add polygon to vector
                            polygon->push_back(p);
                        }

                        fcoordsData.clear();

                        fcoordsData.push_back(p.getCoords()[0][0]);
                        fcoordsData.push_back(p.getCoords()[0][1]);
                        fcoordsData.push_back(p.getCoords()[0][2]);
                        fcoordsData.push_back(p.getCoords()[2][0]);
                        fcoordsData.push_back(p.getCoords()[2][1]);
                        fcoordsData.push_back(p.getCoords()[2][2]);

                        if (fillData[0] == POLY_PATCH)
                        {
                            normData.clear();

                            normData.push_back(p.getNorms()[0][0]);
                            normData.push_back(p.getNorms()[0][1]);
                            normData.push_back(p.getNorms()[0][2]);
                            normData.push_back(p.getNorms()[2][0]);
                            normData.push_back(p.getNorms()[2][1]);
                            normData.push_back(p.getNorms()[2][2]);
                        }
                    }
                }
            }
            // spheres
            else if (fillData[0] == SPHERE)
            {
                p.setType(SPHERE);
                std::vector<float> fcoordsData;

                for (unsigned int i = 1; i < fillData.size(); i ++)
                {
                    fcoordsData.push_back(stof(fillData[i]));
                }

                p.setscoords(fcoordsData);
                polygon->push_back(p);
            }
        }
    }

    nff_file.close();
    return 0;
}

void camCoords(std::vector<Eigen::Vector3d> viewpoint, Camera *origin)
{
    Eigen::Vector3d w = (viewpoint[AT] - viewpoint[FROM]).normalized();
    Eigen::Vector3d u = (viewpoint[UP].cross(w)).normalized();
    Eigen::Vector3d v = (u.cross(w));

    origin->setW(w);
    origin->setU(u);
    origin->setV(v);
}

void clipping(double r, double l, double t, double b, double n, double f, std::vector<Polygon> *polygon, int polynum)
{
    std::vector<double> in;
    std::vector<double> out;
    std::vector<Eigen::Vector3d> vex;

    double highest = -1000, lowest = 1512, rightmost = -1000, leftmost = 1512, nearest = -1;
    for (unsigned int i = 0; i < polygon[0][polynum].getMCoords().size() / 2; i++)
    {
        if (polygon[0][polynum].getMCoords()[i * 2](0) / polygon[0][polynum].getMCoords()[i * 2](3) > rightmost)
        {
            rightmost = polygon[0][polynum].getMCoords()[i * 2](0) / polygon[0][polynum].getMCoords()[i * 2](3);
        }
        if (polygon[0][polynum].getMCoords()[i * 2](0) / polygon[0][polynum].getMCoords()[i * 2](3) < leftmost)
        {
            leftmost = polygon[0][polynum].getMCoords()[i + 2](0) / polygon[0][polynum].getMCoords()[i * 2](3);
        }
        if (polygon[0][polynum].getMCoords()[i * 2](1) / polygon[0][polynum].getMCoords()[i * 2](3) > highest)
        {
            highest = polygon[0][polynum].getMCoords()[i * 2](1) / polygon[0][polynum].getMCoords()[i * 2](3);
        }
        if (polygon[0][polynum].getMCoords()[i * 2](1) / polygon[0][polynum].getMCoords()[i * 2](3) < lowest)
        {
            lowest = polygon[0][polynum].getMCoords()[i * 2](1) / polygon[0][polynum].getMCoords()[i * 2](3);
        }
        if (polygon[0][polynum].getMCoords()[i * 2](2) / polygon[0][polynum].getMCoords()[i * 2](3) > nearest)
        {
            nearest = polygon[0][polynum].getMCoords()[i * 2](2) / polygon[0][polynum].getMCoords()[i * 2](3);
        }

        if (rightmost < 512 && leftmost > 0 && highest < 512 && lowest > 0)
        {
            in.push_back(i);
        }
        else
        {
            out.push_back(i);
        }
    }

    for (unsigned int i = 0; i < in.size(); i++)
    {
        Eigen::Vector3d N; N << polygon[0][polynum].getMCoords()[in[i] * 2](0) / polygon[0][polynum].getMCoords()[in[i] * 2](3), 
                                polygon[0][polynum].getMCoords()[in[i] * 2](1) / polygon[0][polynum].getMCoords()[in[i] * 2](3), 
                                polygon[0][polynum].getMCoords()[in[i] * 2](2) / polygon[0][polynum].getMCoords()[in[i] * 2](3);
        vex.push_back(N);
    }

    if (out.size() == 0)
    {
        return;
    }
    else if (in.size() == 0)
    {
        polygon[0][polynum].setDel(true);
    }
    else
    {
        for (unsigned int i = 0; i < in.size(); i++)
        {
            for (unsigned int j = 0; j < out.size(); j++)
            {
                // Right
                if (polygon[0][polynum].getMCoords()[out[j] * 2](0) / polygon[0][polynum].getMCoords()[out[j] * 2](3) < 0)
                {
                    double t = abs(polygon[0][polynum].getMCoords()[out[j] * 2](0) / polygon[0][polynum].getMCoords()[out[j] * 2](3)) / 
                        abs(polygon[0][polynum].getMCoords()[out[j] * 2](0) / polygon[0][polynum].getMCoords()[out[j] * 2](3) - polygon[0][polynum].getMCoords()[in[j] * 2](0) / polygon[0][polynum].getMCoords()[in[j] * 2](3));
                    
                    double ux = polygon[0][polynum].getMCoords()[in[j] * 2](0) / polygon[0][polynum].getMCoords()[in[j] * 2](3) - polygon[0][polynum].getMCoords()[out[j] * 2](0) / polygon[0][polynum].getMCoords()[out[j] * 2](3);
                    double uy = polygon[0][polynum].getMCoords()[in[j] * 2](1) / polygon[0][polynum].getMCoords()[in[j] * 2](3) - polygon[0][polynum].getMCoords()[out[j] * 2](1) / polygon[0][polynum].getMCoords()[out[j] * 2](3);
                    double uz = polygon[0][polynum].getMCoords()[in[j] * 2](2) / polygon[0][polynum].getMCoords()[in[j] * 2](3) - polygon[0][polynum].getMCoords()[out[j] * 2](2) / polygon[0][polynum].getMCoords()[out[j] * 2](3);
                    double nx = polygon[0][polynum].getMCoords()[out[j] * 2](0) / polygon[0][polynum].getMCoords()[out[j] * 2](3) + t * ux;
                    double ny = polygon[0][polynum].getMCoords()[out[j] * 2](1) / polygon[0][polynum].getMCoords()[out[j] * 2](3) + t * uy;
                    double nz = polygon[0][polynum].getMCoords()[out[j] * 2](2) / polygon[0][polynum].getMCoords()[out[j] * 2](3) + t * uz;
                    Eigen::Vector3d N; N << nx, ny, nz;
                    vex.push_back(N);
                }
                // Left
                if (polygon[0][polynum].getMCoords()[out[j] * 2](0) / polygon[0][polynum].getMCoords()[out[j] * 2](3) > 512)
                {
                    double t = abs(polygon[0][polynum].getMCoords()[out[j] * 2](0) / polygon[0][polynum].getMCoords()[out[j] * 2](3) - 512) / 
                        abs(polygon[0][polynum].getMCoords()[out[j] * 2](0) / polygon[0][polynum].getMCoords()[out[j] * 2](3) - polygon[0][polynum].getMCoords()[in[j] * 2](0) / polygon[0][polynum].getMCoords()[in[j] * 2](3));
                    
                    double ux = polygon[0][polynum].getMCoords()[in[j] * 2](0) / polygon[0][polynum].getMCoords()[in[j] * 2](3) - polygon[0][polynum].getMCoords()[out[j] * 2](0) / polygon[0][polynum].getMCoords()[out[j] * 2](3);
                    double uy = polygon[0][polynum].getMCoords()[in[j] * 2](1) / polygon[0][polynum].getMCoords()[in[j] * 2](3) - polygon[0][polynum].getMCoords()[out[j] * 2](1) / polygon[0][polynum].getMCoords()[out[j] * 2](3);
                    double uz = polygon[0][polynum].getMCoords()[in[j] * 2](2) / polygon[0][polynum].getMCoords()[in[j] * 2](3) - polygon[0][polynum].getMCoords()[out[j] * 2](2) / polygon[0][polynum].getMCoords()[out[j] * 2](3);
                    double nx = polygon[0][polynum].getMCoords()[out[j] * 2](0) / polygon[0][polynum].getMCoords()[out[j] * 2](3) + t * ux;
                    double ny = polygon[0][polynum].getMCoords()[out[j] * 2](1) / polygon[0][polynum].getMCoords()[out[j] * 2](3) + t * uy;
                    double nz = polygon[0][polynum].getMCoords()[out[j] * 2](2) / polygon[0][polynum].getMCoords()[out[j] * 2](3) + t * uz;
                    Eigen::Vector3d N; N << nx, ny, nz;
                    vex.push_back(N);
                }
                // Top
                if (polygon[0][polynum].getMCoords()[out[j] * 2](1) / polygon[0][polynum].getMCoords()[out[j] * 2](3) < 0)
                {
                    double t = abs(polygon[0][polynum].getMCoords()[out[j] * 2](1) / polygon[0][polynum].getMCoords()[out[j] * 2](3)) / 
                        abs(polygon[0][polynum].getMCoords()[out[j] * 2](1) / polygon[0][polynum].getMCoords()[out[j] * 2](3) - polygon[0][polynum].getMCoords()[in[j] * 2](1) / polygon[0][polynum].getMCoords()[in[j] * 2](3));
                    
                    double ux = polygon[0][polynum].getMCoords()[in[j] * 2](0) / polygon[0][polynum].getMCoords()[in[j] * 2](3) - polygon[0][polynum].getMCoords()[out[j] * 2](0) / polygon[0][polynum].getMCoords()[out[j] * 2](3);
                    double uy = polygon[0][polynum].getMCoords()[in[j] * 2](1) / polygon[0][polynum].getMCoords()[in[j] * 2](3) - polygon[0][polynum].getMCoords()[out[j] * 2](1) / polygon[0][polynum].getMCoords()[out[j] * 2](3);
                    double uz = polygon[0][polynum].getMCoords()[in[j] * 2](2) / polygon[0][polynum].getMCoords()[in[j] * 2](3) - polygon[0][polynum].getMCoords()[out[j] * 2](2) / polygon[0][polynum].getMCoords()[out[j] * 2](3);
                    double nx = polygon[0][polynum].getMCoords()[out[j] * 2](0) / polygon[0][polynum].getMCoords()[out[j] * 2](3) + t * ux;
                    double ny = polygon[0][polynum].getMCoords()[out[j] * 2](1) / polygon[0][polynum].getMCoords()[out[j] * 2](3) + t * uy;
                    double nz = polygon[0][polynum].getMCoords()[out[j] * 2](2) / polygon[0][polynum].getMCoords()[out[j] * 2](3) + t * uz;
                    Eigen::Vector3d N; N << nx, ny, nz;
                    vex.push_back(N);
                }
                // Bottom
                if (polygon[0][polynum].getMCoords()[out[j] * 2](1) / polygon[0][polynum].getMCoords()[out[j] * 2](3) < 0)
                {
                    double t = abs(polygon[0][polynum].getMCoords()[out[j] * 2](1) / polygon[0][polynum].getMCoords()[out[j] * 2](3)) / 
                        abs(polygon[0][polynum].getMCoords()[out[j] * 2](1) / polygon[0][polynum].getMCoords()[out[j] * 2](3) - polygon[0][polynum].getMCoords()[in[j] * 2](1) / polygon[0][polynum].getMCoords()[in[j] * 2](3));
                    
                    double ux = polygon[0][polynum].getMCoords()[in[j] * 2](0) / polygon[0][polynum].getMCoords()[in[j] * 2](3) - polygon[0][polynum].getMCoords()[out[j] * 2](0) / polygon[0][polynum].getMCoords()[out[j] * 2](3);
                    double uy = polygon[0][polynum].getMCoords()[in[j] * 2](1) / polygon[0][polynum].getMCoords()[in[j] * 2](3) - polygon[0][polynum].getMCoords()[out[j] * 2](1) / polygon[0][polynum].getMCoords()[out[j] * 2](3);
                    double uz = polygon[0][polynum].getMCoords()[in[j] * 2](2) / polygon[0][polynum].getMCoords()[in[j] * 2](3) - polygon[0][polynum].getMCoords()[out[j] * 2](2) / polygon[0][polynum].getMCoords()[out[j] * 2](3);
                    double nx = polygon[0][polynum].getMCoords()[out[j] * 2](0) / polygon[0][polynum].getMCoords()[out[j] * 2](3) + t * ux;
                    double ny = polygon[0][polynum].getMCoords()[out[j] * 2](1) / polygon[0][polynum].getMCoords()[out[j] * 2](3) + t * uy;
                    double nz = polygon[0][polynum].getMCoords()[out[j] * 2](2) / polygon[0][polynum].getMCoords()[out[j] * 2](3) + t * uz;
                    Eigen::Vector3d N; N << nx, ny, nz;
                    vex.push_back(N);
                }
            }
        }

        // Split more poly if vex.size() > 3
        if (vex.size() > 3)
        {
            for (unsigned int i = 0; i < vex.size(); i++)
            {
                if (i >= 2)
                {
                    // first tri
                    Polygon p;
                    if (i == 2)
                    {
                        p.setType(polygon[0][polynum].getType());
                        Eigen::Vector4d v0; v0 << vex[0], 1; 
                        Eigen::Vector4d v1; v1 << vex[1], 1; 
                        Eigen::Vector4d v2; v2 << vex[i], 1; 
                        p.setMCoords(v0);
                        p.setMCoords(v1);
                        p.setMCoords(v2);
                        std::vector<float> col;
                        for (int j = 0; j < 3; j++)
                        {
                            col.push_back(polygon[0][polynum].getVColor(j));
                        }
                        p.setVColor(col);
                        col.clear();
                        for (int j = 3; j < 6; j++)
                        {
                            col.push_back(polygon[0][polynum].getVColor(j));
                        }
                        p.setVColor(col);
                        col.clear();
                        for (int j = 6; j < 9; j++)
                        {
                            col.push_back(polygon[0][polynum].getVColor(j));
                        }
                        p.setVColor(col);
                    }
                    else
                    {
                        p.setType(polygon[0][polynum].getType());
                        Eigen::Vector4d v0; v0 << vex[0], 1; 
                        Eigen::Vector4d v1; v1 << vex[i - 1], 1; 
                        Eigen::Vector4d v2; v2 << vex[i], 1; 
                        p.setMCoords(v0);
                        p.setMCoords(v1);
                        p.setMCoords(v2);
                        std::vector<float> col;
                        for (int j = 0; j < 3; j++)
                        {
                            col.push_back(polygon[0][polynum].getVColor(j));
                        }
                        p.setVColor(col);
                        col.clear();
                        for (int j = 3; j < 6; j++)
                        {
                            col.push_back(polygon[0][polynum].getVColor(j));
                        }
                        p.setVColor(col);
                        col.clear();
                        for (int j = 6; j < 9; j++)
                        {
                            col.push_back(polygon[0][polynum].getVColor(j));
                        }
                        p.setVColor(col);
                    }
                    polygon->push_back(p);
                }
            }
            polygon[0][polynum].setDel(true);
        }
    }
}

void processVertex(std::vector<Polygon> *polygon, std::vector<Eigen::Vector3d> light, Eigen::Vector3d background, Eigen::Vector3d origin, Eigen::MatrixXd M,
                    double right, double left, double top, double bottom, double near, double far)
{
    // Loop each polygon
    const int num = polygon->size();
    for (int i = 0; i < num; i++)
    {
        // Loop each vertex in polygon
        for (unsigned int j = 0; j < polygon[0][i].getCoords().size(); j++)
        {
            Eigen::Vector3d N;
            Eigen::Vector3d vd = (polygon[0][i].getCoords()[j] - origin).normalized();
            Eigen::Vector3d L;

            double diffuse = 0;
            double red = 0;
            double green = 0;
            double blue = 0;
            double specular = 0;

            // Get normals
            if (polygon[0][i].getType() == POLYGON)
            {
                N = (polygon[0][i].getCoords()[1] - polygon[0][i].getCoords()[0]).cross(polygon[0][i].getCoords()[2] - polygon[0][i].getCoords()[0]).normalized();
            }
            else if (polygon[0][i].getType() == POLY_PATCH)
            {
                N = polygon[0][i].getNorms()[j].normalized();
            }

            // Calculate colors based on lights
            for (unsigned int k = 0; k < light.size(); k++)
            {
                L << (light[k](0) - polygon[0][i].getCoords()[j](0)), (light[k](1) - polygon[0][i].getCoords()[j](1)), (light[k](2) - polygon[0][i].getCoords()[j](2));
                L.normalize();

                diffuse = max(0, N.dot(L));
                Eigen::Vector3d H((L - vd).normalized());
                specular = pow(max(0, N.dot(H)), polygon[0][i].getColor(5));
                red += ((polygon[0][i].getColor(3) * polygon[0][i].getColor(0) * diffuse + polygon[0][i].getColor(4) * specular) * (1 / sqrt(light.size())));
                green += ((polygon[0][i].getColor(3) * polygon[0][i].getColor(1) * diffuse + polygon[0][i].getColor(4) * specular) * (1 / sqrt(light.size())));
                blue += ((polygon[0][i].getColor(3) * polygon[0][i].getColor(2) * diffuse + polygon[0][i].getColor(4) * specular) * (1 / sqrt(light.size())));
            }

            // set vertex color
            std::vector<float> vcol;
            vcol.push_back(red);
            vcol.push_back(green);
            vcol.push_back(blue);
            polygon[0][i].setVColor(vcol);

            Eigen::Vector4d tmp; tmp << polygon[0][i].getCoords()[j], 1;
            Eigen::Vector4d mc = M * tmp;
            polygon[0][i].setMCoords(mc);

        }
        // clipping(right, left, top, bottom, near, far, polygon, i);
    }
    // for (unsigned int i = 0; i < polygon[0].size(); i++)
    // {
    //     if (polygon[0][i].getDel())
    //     {
    //         polygon[0].erase(polygon->begin() + i);
    //         i--;
    //     }
    // }
}

void rasterization(std::vector<Polygon> *polygon, std::vector<std::vector<HitRecord> > *hr)
{
    for (unsigned int i = 0; i < polygon[0].size(); i++)
    {
        int xMin = 1000, xMax = 0, yMin = 1000, yMax = 0;
        for (unsigned int j = 0; j < polygon[0][i].getMCoords().size(); j++)
        {
            if (polygon[0][i].getMCoords()[j](0) / polygon[0][i].getMCoords()[j](3) < xMin)
            {
                xMin = floor(polygon[0][i].getMCoords()[j](0) / polygon[0][i].getMCoords()[j](3));
            }
            if (polygon[0][i].getMCoords()[j](0) / polygon[0][i].getMCoords()[j](3) > xMax)
            {
                xMax = ceil(polygon[0][i].getMCoords()[j](0) / polygon[0][i].getMCoords()[j](3));
            }
            if (polygon[0][i].getMCoords()[j](1) / polygon[0][i].getMCoords()[j](3) < yMin)
            {
                yMin = floor(polygon[0][i].getMCoords()[j](1) / polygon[0][i].getMCoords()[j](3));
            }
            if (polygon[0][i].getMCoords()[j](1) / polygon[0][i].getMCoords()[j](3) > yMax)
            {
                yMax = ceil(polygon[0][i].getMCoords()[j](1) / polygon[0][i].getMCoords()[j](3));
            }
        }

        for (int y = yMin; y < yMax; y++)
        {
            for (int x = xMin; x < xMax; x++)
            {
                double x0, x1, x2, y0, y1, y2, z0, z1, z2;
                x0 = polygon[0][i].getMCoords()[0](0) / polygon[0][i].getMCoords()[0](3);
                x1 = polygon[0][i].getMCoords()[2](0) / polygon[0][i].getMCoords()[2](3);
                x2 = polygon[0][i].getMCoords()[4](0) / polygon[0][i].getMCoords()[4](3);
                y0 = polygon[0][i].getMCoords()[0](1) / polygon[0][i].getMCoords()[0](3);
                y1 = polygon[0][i].getMCoords()[2](1) / polygon[0][i].getMCoords()[2](3);
                y2 = polygon[0][i].getMCoords()[4](1) / polygon[0][i].getMCoords()[4](3);
                z0 = polygon[0][i].getMCoords()[0](2) / polygon[0][i].getMCoords()[0](3);
                z1 = polygon[0][i].getMCoords()[2](2) / polygon[0][i].getMCoords()[2](3);
                z2 = polygon[0][i].getMCoords()[4](2) / polygon[0][i].getMCoords()[4](3);


                // f(x, y) = (y0 - y1)x + (x1 - x0)y + x0y1 - x1y0 = 0
                double a = ((y1 - y2) * x + (x2 - x1) * y + x1 * y2 - x2 * y1) / ((y1 - y2) * x0 + (x2 - x1) * y0 + x1 * y2 - x2 * y1);
                double b = ((y2 - y0) * x + (x0 - x2) * y + x2 * y0 - x0 * y2) / ((y2 - y0) * x1 + (x0 - x2) * y1 + x2 * y0 - x0 * y2);
                double g = ((y0 - y1) * x + (x1 - x0) * y + x0 * y1 - x1 * y0) / ((y0 - y1) * x2 + (x1 - x0) * y2 + x0 * y1 - x1 * y0);

                // Color
                if (a > 0 && b > 0 && g > 0)
                {
                    Eigen::Vector3d c1; c1 << polygon[0][i].getVColor(0), polygon[0][i].getVColor(1), polygon[0][i].getVColor(2);
                    Eigen::Vector3d c2; c2 << polygon[0][i].getVColor(3), polygon[0][i].getVColor(4), polygon[0][i].getVColor(5);
                    Eigen::Vector3d c3; c3 << polygon[0][i].getVColor(6), polygon[0][i].getVColor(7), polygon[0][i].getVColor(8);
                    double z = a * z0 + b * z1 + g * z2;
                    
                    Eigen::Vector3d c = a * c1 + b * c2 + g * c3;
                    c(0) > 1 ? c(0) = 1 : c(0);
                    c(1) > 1 ? c(1) = 1 : c(1);
                    c(2) > 1 ? c(2) = 1 : c(2);

                    if (x < 512 && x >= 0 && y < 512 && y >= 0)
                    {
                        if (z > hr[0][x][511 - y].getZ())
                        {
                            hr[0][x][511 - y].setColor(c);
                            hr[0][x][511 - y].setZ(z);
                        }
                    }
                }
            }
        }
    }
}

void fragmentProcessing()
{

}

int main(int argc, char *argv[])
{
    Eigen::Vector3d background;
    std::vector<Eigen::Vector3d> viewpoint;
    std::vector<Polygon> polygon;
    std::vector<Eigen::Vector3d> lights;
    Camera cCoords;
    std::vector<std::vector<HitRecord> > hitRecord;
    std::string renderImage = argv[1];
    std::cout << "Parsing nff file: " << renderImage << "\n";
    parseNff(renderImage, &background, &viewpoint, &polygon, &lights);
    std::cout << "Done parsing\nTotal polygons: " << polygon.size() << "\nStarting Vertex processing\n";

    const int height = viewpoint[RES](1, 0);
    const int width = viewpoint[RES](0, 0);

    // set size of hr and set initial color to bg
    hrInit(&hitRecord, width, height, background);

    unsigned char pixel[height][width][3];

    // Compute cam coords
    camCoords(viewpoint, &cCoords);

    double near = -viewpoint[HITHER][0];
    double far = near * 1000;
    double h= (tan((viewpoint[ANGLE][0] * M_PI / 180) / 2));
    double right = h;
    double left = -h;
    double top = right / (width / height);
    double bottom = left / (width / height);

    Eigen::MatrixXd Mvp(4, 4);
    Eigen::MatrixXd Morth(4, 4);
    Eigen::MatrixXd Mcam(4, 4);
    Eigen::MatrixXd P(4, 4);

    // Create transformation Matrix and create transformation Matrix
    matrixInit(&Mvp, &Morth, &Mcam, &P, cCoords, viewpoint[FROM], near, far, right, left, top, bottom, width, height);
    Eigen::MatrixXd M(4, 4);
    M = Mvp * Morth * P * Mcam;

    processVertex(&polygon, lights, background, viewpoint[FROM], M, right * 1000, left * 1000, top * 1000, bottom * 1000, near, far);
    std::cout << "Vertex processing finished\nStarting rasterization\n";

    rasterization(&polygon, &hitRecord);

    std::cout << "Rasterization finished\nBlending\n";
    // Blending
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            pixel[i][j][0] = hitRecord[j][i].getColor()(0, 0) * 255;
            pixel[i][j][1] = hitRecord[j][i].getColor()(1, 0) * 255;
            pixel[i][j][2] = hitRecord[j][i].getColor()(2, 0) * 255;
        }
    }

    FILE *file = fopen("output.ppm", "wb");
    fprintf(file, "P6\n%d %d\n%d\n", width, height, 255);
    fwrite(pixel, 1, height*width*3, file);
    fclose(file);
    std::cout << "Done\n\n";
    return 0;
}