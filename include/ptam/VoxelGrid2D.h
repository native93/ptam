#ifndef _VOXELGRID2D_H_
#define _VOXELGRID2D_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <ros/console.h>

typedef pcl::PointXYZ PointT;


#define VOXELGRID2D_CORNER_TOP_LEFT 1
#define VOXELGRID2D_CORNER_TOP_RIGHT 2
#define VOXELGRID2D_CORNER_BOTTOM_LEFT 3
#define VOXELGRID2D_CORNER_BOTTOM_RIGHT 4

class VoxelGrid2D {
    private:
        int _width;
        int _height;
        PointT _min_pt;
        PointT _max_pt;
        float _cellsize;
        uint8_t* _grid;
        void getBoundingBox(pcl::PointCloud<PointT>::Ptr &cloud);
    public:
        VoxelGrid2D(pcl::PointCloud<PointT>::Ptr &cloud, float cellsize);
        ~VoxelGrid2D();
        void getGridLocation(PointT pt, int &i, int &j);
        bool isOccupied(int i, int j);
        void setOccupied(int i, int j);
        void setUnOccupied(int i, int j);
        PointT getGridCorner(int i, int j, int c_no);
        int getWidth() { return _width; }
        int getHeight() { return _height; }
        void printAllCorners();
        float getResolution() const;
};

#endif
