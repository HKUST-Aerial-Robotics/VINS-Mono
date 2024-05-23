#include "data_generator.h"

#define SEED 1
#define Y_COS 2
#define Z_COS 2 * 2
#define IMU_NOISE 0
#define IMG_NOISE 0
#define BIAS_ACC 0
#define BIAS_GYR 1

DataGenerator::DataGenerator()
{
    srand(SEED);
    t = 0;
    current_id = 0;

    for (int i = 0; i < NUM_POINTS; i++)
    {
        pts[i * 3 + 0] = rand() % (6 * MAX_BOX) - 3 * MAX_BOX;
        pts[i * 3 + 1] = rand() % (6 * MAX_BOX) - 3 * MAX_BOX;
        pts[i * 3 + 2] = rand() % (6 * MAX_BOX) - 3 * MAX_BOX;
        cout << "pts i " << i << " " << pts[i * 3 + 0] << " " << pts[i * 3 + 1] << " " << pts[i * 3 + 2] << endl;
    }

    ap[0] = Vector3d(MAX_BOX, -MAX_BOX, MAX_BOX);
    ap[1] = Vector3d(-MAX_BOX, MAX_BOX, MAX_BOX);
    ap[2] = Vector3d(-MAX_BOX, -MAX_BOX, -MAX_BOX);
    ap[3] = Vector3d(MAX_BOX, MAX_BOX, -MAX_BOX);

    Ric[0] << 0, 0, -1,
        -1, 0, 0,
        0, 1, 0;
    Tic[0] << 0.0, 0.2, 0.6;
    if (NUMBER_OF_CAMERA >= 2)
    {
        Ric[1] = Ric[0];
        Tic[1] << 0.00, 0.00, 0.30;
    }
    if (NUMBER_OF_CAMERA >= 3)
    {
        Ric[2] << 0, 1, 0,
            -1, 0, 0,
            0, 0, 1;
        Tic[2] << 0.00, 0.00, 0.15;
    }

    acc_cov = 0.01 * 0.01 * Matrix3d::Identity();
    gyr_cov = 0.001 * 0.001 * Matrix3d::Identity();
    pts_cov = (0.3 / 460) * (0.3 / 460) * Matrix2d::Identity();

    generator = default_random_engine(SEED);
    distribution = normal_distribution<double>(0.0, 1);
    Axis[0] = Vector3d(10, 0, 0);
    Axis[1]= Vector3d(0, 10, 0);
    Axis[2] = Vector3d(0, 0, 10);
    Axis[3] = Vector3d(-10, 0, 0);
    Axis[4]= Vector3d(0, -10, 0);
    Axis[5] = Vector3d(0, 0, -10);
}

void DataGenerator::update()
{
    t += 1.0 / FREQ;
}

double DataGenerator::getTime()
{
    return t;
}

Vector3d DataGenerator::getPoint(int i)
{
    return Vector3d(pts[3 * i], pts[3 * i + 1], pts[3 * i + 2]);
}

Vector3d DataGenerator::getAP(int i)
{
    return ap[i];
}

Vector3d DataGenerator::getPosition()
{
    double x, y, z;
    if (t < MAX_TIME)
    {
        x = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * M_PI);
        y = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * M_PI * Y_COS);
        z = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(t / MAX_TIME * M_PI * Z_COS);
    }
    else if (t >= MAX_TIME && t < 2 * MAX_TIME)
    {
        x = MAX_BOX / 2.0 - MAX_BOX / 2.0;
        y = MAX_BOX / 2.0 + MAX_BOX / 2.0;
        z = MAX_BOX / 2.0 + MAX_BOX / 2.0;
    }
    else
    {
        double tt = t - 2 * MAX_TIME;
        x = -MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(tt / MAX_TIME * M_PI);
        y = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(tt / MAX_TIME * M_PI * Y_COS);
        z = MAX_BOX / 2.0 + MAX_BOX / 2.0 * cos(tt / MAX_TIME * M_PI * Z_COS);
    }

    return Vector3d(x, y, z);
}

Matrix3d DataGenerator::getRotation()
{
    //return Matrix3d::Identity();
    return (AngleAxisd(30.0 / 180 * M_PI * sin(t / MAX_TIME * M_PI * 2), Vector3d::UnitX()) * AngleAxisd(40.0 / 180 * M_PI * sin(t / MAX_TIME * M_PI * 2), Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ())).toRotationMatrix();
}

Vector3d DataGenerator::getAngularVelocity()
{
    const double delta_t = 0.00001;
    Matrix3d rot = getRotation();
    t += delta_t;
    Matrix3d drot = (getRotation() - rot) / delta_t;
    t -= delta_t;
    Matrix3d skew = rot.inverse() * drot;
    if(IMU_NOISE)
    {
        Vector3d disturb = Vector3d(distribution(generator) * sqrt(gyr_cov(0, 0)),
                                    distribution(generator) * sqrt(gyr_cov(1, 1)),
                                    distribution(generator) * sqrt(gyr_cov(2, 2)));
        return disturb + Vector3d(skew(2, 1), -skew(2, 0), skew(1, 0));
    }
    else
    {
#if BIAS_GYR
        return Vector3d(skew(2, 1) + 0.02, -skew(2, 0) + 0.03, skew(1, 0) + 0.04);
#endif
        return Vector3d(skew(2, 1), -skew(2, 0), skew(1, 0));
    }

}

Vector3d DataGenerator::getVelocity()
{
    double dx, dy, dz;
    if (t < MAX_TIME)
    {
        dx = MAX_BOX / 2.0 * -sin(t / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        dy = MAX_BOX / 2.0 * -sin(t / MAX_TIME * M_PI * Y_COS) * (1.0 / MAX_TIME * M_PI * Y_COS);
        dz = MAX_BOX / 2.0 * -sin(t / MAX_TIME * M_PI * Z_COS) * (1.0 / MAX_TIME * M_PI * Z_COS);
    }
    else if (t >= MAX_TIME && t < 2 * MAX_TIME)
    {
        dx = 0.0;
        dy = 0.0;
        dz = 0.0;
    }
    else
    {
        double tt = t - 2 * MAX_TIME;
        dx = MAX_BOX / 2.0 * -sin(tt / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        dy = MAX_BOX / 2.0 * -sin(tt / MAX_TIME * M_PI * Y_COS) * (1.0 / MAX_TIME * M_PI * Y_COS);
        dz = MAX_BOX / 2.0 * -sin(tt / MAX_TIME * M_PI * Z_COS) * (1.0 / MAX_TIME * M_PI * Z_COS);
    }

    return getRotation().inverse() * Vector3d(dx, dy, dz);
}

Vector3d DataGenerator::getLinearAcceleration()
{
    double ddx, ddy, ddz;
    if (t < MAX_TIME)
    {
        ddx = MAX_BOX / 2.0 * -cos(t / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        ddy = MAX_BOX / 2.0 * -cos(t / MAX_TIME * M_PI * Y_COS) * (1.0 / MAX_TIME * M_PI * Y_COS) * (1.0 / MAX_TIME * M_PI * Y_COS);
        ddz = MAX_BOX / 2.0 * -cos(t / MAX_TIME * M_PI * Z_COS) * (1.0 / MAX_TIME * M_PI * Z_COS) * (1.0 / MAX_TIME * M_PI * Z_COS);
    }
    else if (t >= MAX_TIME && t < 2 * MAX_TIME)
    {
        ddx = 0.0;
        ddy = 0.0;
        ddz = 0.0;
    }
    else
    {
        double tt = t - 2 * MAX_TIME;
        ddx = MAX_BOX / 2.0 * -cos(tt / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI) * (1.0 / MAX_TIME * M_PI);
        ddy = MAX_BOX / 2.0 * -cos(tt / MAX_TIME * M_PI * Y_COS) * (1.0 / MAX_TIME * M_PI * Y_COS) * (1.0 / MAX_TIME * M_PI * Y_COS);
        ddz = MAX_BOX / 2.0 * -cos(tt / MAX_TIME * M_PI * Z_COS) * (1.0 / MAX_TIME * M_PI * Z_COS) * (1.0 / MAX_TIME * M_PI * Z_COS);
    }
    if(IMU_NOISE)
    {
        Vector3d disturb = Vector3d(distribution(generator) * sqrt(acc_cov(0, 0)),
                                    distribution(generator) * sqrt(acc_cov(1, 1)),
                                    distribution(generator) * sqrt(acc_cov(2, 2)));
        return getRotation().inverse() * (disturb + Vector3d(ddx, ddy, ddz + 9.805));
    }
    else
    {
#if BIAS_ACC
        return getRotation().inverse() * Vector3d(ddx, ddy, ddz + 9.805) + Vector3d(0.01, 0.02, 0.03);
#endif
        return getRotation().inverse() * Vector3d(ddx, ddy, ddz + 9.805);
        
    }

}

vector<pair<int, Vector3d>> DataGenerator::getImage()
{
    vector<pair<int, Vector3d>> image;
    Vector3d position = getPosition();
    Matrix3d quat = getRotation();
    printf("max: %d\n", current_id);

    vector<int> ids[NUMBER_OF_CAMERA], gr_ids[NUMBER_OF_CAMERA];
    vector<Vector3d> cur_pts[NUMBER_OF_CAMERA];
    for (int k = 0; k < NUMBER_OF_CAMERA; k++)
    {
        for (int i = 0; i < NUM_POINTS; i++)
        {
            double xx = pts[i * 3 + 0] - position(0);
            double yy = pts[i * 3 + 1] - position(1);
            double zz = pts[i * 3 + 2] - position(2);
            Vector3d local_point = Ric[k].inverse() * (quat.inverse() * Vector3d(xx, yy, zz) - Tic[k]);
            xx = local_point(0);
            yy = local_point(1);
            zz = local_point(2);

            if (zz > 0.0 && std::fabs(atan2(xx, zz)) <= M_PI * FOV / 2 / 180 && std::fabs(atan2(yy, zz)) <= M_PI * FOV / 2 / 180)
            {
                xx = xx / zz;
                yy = yy / zz;
                zz = zz / zz;
#if IMG_NOISE
                xx += distribution(generator) * sqrt(pts_cov(0, 0));
                yy += distribution(generator) * sqrt(pts_cov(1, 1));
#endif

                int n_id = before_feature_id[k].find(i) == before_feature_id[k].end() ? -1 /*current_id++*/ : before_feature_id[k][i];
                ids[k].push_back(n_id);
                gr_ids[k].push_back(i);
                cur_pts[k].push_back(Vector3d(xx, yy, zz));
            }
        }

        for (int i = 0; i < 6; i++)
        {
            output_Axis[i].clear();
            Vector3d local_point;
            local_point = Ric[k].inverse() * (quat.inverse() * (Axis[i] - position) - Tic[k]);
            double xx = local_point(0);
            double yy = local_point(1);
            double zz = local_point(2);
            if(zz > 0.0 && std::fabs(atan2(xx, zz)) <= M_PI * FOV / 2 / 180 && std::fabs(atan2(yy, zz)) <= M_PI * FOV / 2 / 180)
            {
                xx = xx / zz;
                yy = yy / zz;
                zz = zz / zz;
                output_Axis[i].push_back(Vector3d(xx, yy, zz));
                local_point = Ric[k].inverse() * (quat.inverse() * (Axis[i] + Vector3d(1, 0, 0) - position) - Tic[k]);
                xx = local_point(0);
                yy = local_point(1);
                zz = local_point(2);
                xx = xx / zz;
                yy = yy / zz;
                zz = zz / zz;
                output_Axis[i].push_back(Vector3d(xx, yy, zz));
                local_point = Ric[k].inverse() * (quat.inverse() * (Axis[i] + Vector3d(0, 1, 0) - position) - Tic[k]);
                xx = local_point(0);
                yy = local_point(1);
                zz = local_point(2);
                xx = xx / zz;
                yy = yy / zz;
                zz = zz / zz;
                output_Axis[i].push_back(Vector3d(xx, yy, zz));
                local_point = Ric[k].inverse() * (quat.inverse() * (Axis[i] + Vector3d(0, 0, 1) - position) - Tic[k]);
                xx = local_point(0);
                yy = local_point(1);
                zz = local_point(2);
                xx = xx / zz;
                yy = yy / zz;
                zz = zz / zz;
                output_Axis[i].push_back(Vector3d(xx, yy, zz));
            }
        }
    }

    output_gr_pts.clear();
    for (auto i : gr_ids[0])
    {
        output_gr_pts.emplace_back(pts[i * 3 + 0], pts[i * 3 + 1], pts[i * 3 + 2]);
    }

    //for (int k = 0; k < 1/* NUMBER_OF_CAMERA - 1 */; k++)
    //{
    //    for (int i = 0; i < int(ids[k].size()); i++)
    //    {
    //        if (ids[k][i] != -1)
    //            continue;
    //        for (int j = 0; j < int(ids[k + 1].size()); j++)
    //            if (ids[k + 1][j] == -1 && gr_ids[k][i] == gr_ids[k + 1][j])
    //                ids[k][i] = ids[k + 1][j] = current_id++;
    //    }
    //}

    for (int k = 0; k < NUMBER_OF_CAMERA; k++)
    {
        for (int i = 0; i < int(ids[k].size()); i++)
        {
            if (ids[k][i] == -1)
                ids[k][i] = current_id++;
            current_feature_id[k][gr_ids[k][i]] = ids[k][i];
        }
        std::swap(before_feature_id[k], current_feature_id[k]);
        current_feature_id[k].clear();
    }

    for (int k = 0; k < NUMBER_OF_CAMERA; k++)
    {
        if (k != 1)
        {
            for (unsigned int i = 0; i < ids[k].size(); i++)
                image.push_back(make_pair(ids[k][i] * NUMBER_OF_CAMERA + k, cur_pts[k][i]));
        }
        else if (k == 1)
        {
            for (unsigned int i = 0; i < ids[k].size(); i++)
            {
                if (before_feature_id[0].find(gr_ids[k][i]) != before_feature_id[0].end())
                    image.push_back(make_pair(before_feature_id[0][gr_ids[k][i]] * NUMBER_OF_CAMERA + k, cur_pts[k][i]));
            }
        }
    }
    return image;
}

vector<Vector3d> DataGenerator::getCloud()
{
    vector<Vector3d> cloud;
    for (int i = 0; i < NUM_POINTS; i++)
    {
        double xx = pts[i * 3 + 0];
        double yy = pts[i * 3 + 1];
        double zz = pts[i * 3 + 2];
        cloud.push_back(Vector3d(xx, yy, zz));
    }
    return cloud;
}
