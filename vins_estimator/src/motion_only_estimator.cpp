#include "motion_only_estimator.h"

MotionOnlyEstimator::MotionOnlyEstimator()
{
    clearState();
}

void MotionOnlyEstimator::setIMUModel()
{
    PerspectiveFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
}

void MotionOnlyEstimator::clearState()
{
    for (int i = 0; i <= PNP_SIZE; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        pre_integrations[i] = nullptr;
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();
        
        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
        features[i].clear();
        find_solved[i] = false;
    }  
    frame_count = 0;
    first_imu = false;
}

void MotionOnlyEstimator::setInit(VINS_RESULT vins_result)
{
    tic = vins_result.tic;
    ric = vins_result.ric;
    g = vins_result.g;
    for (int i = 0; i <= PNP_SIZE; i++)
    {
        Bas[i] = vins_result.Ba;
        Bgs[i] = vins_result.Bg;
        if(Headers[i] == vins_result.header)
        {
            find_solved[i] = true;
            find_solved_vins[i].P = vins_result.P;
            find_solved_vins[i].R = vins_result.R;
            find_solved_vins[i].V = vins_result.V;
            Ps[i] = vins_result.P;
            Rs[i] = vins_result.R;
            Vs[i] = vins_result.V;
            ROS_DEBUG("pnp find new index %d", i);
        }
    }
}

void MotionOnlyEstimator::updateFeatures(vector<IMG_MSG_LOCAL> &feature_msg)
{
    for(int i = 0; i < frame_count; i++)
    {
        int j = 0;
        for (auto &it : feature_msg)
        {
            while(features[i][j].id < it.id)
            {
                j++;
            }
            if(features[i][j].id == it.id)
            {
                features[i][j].position = it.position;
                features[i][j].track_num = it.track_num;
            }
        }
    }
}

void MotionOnlyEstimator::old2new()
{
    for (int i = 0; i <= PNP_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();
        
        para_Speed[i][0] = Vs[i].x();
        para_Speed[i][1] = Vs[i].y();
        para_Speed[i][2] = Vs[i].z();
        
        para_Bias[i][0] = Bas[i].x();
        para_Bias[i][1] = Bas[i].y();
        para_Bias[i][2] = Bas[i].z();
        
        para_Bias[i][3] = Bgs[i].x();
        para_Bias[i][4] = Bgs[i].y();
        para_Bias[i][5] = Bgs[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic.x();
        para_Ex_Pose[i][1] = tic.y();
        para_Ex_Pose[i][2] = tic.z();
        Quaterniond q{ric};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }
}

void MotionOnlyEstimator::new2old()
{
    int solved_index = 0;
    for(int i = 0; i < PNP_SIZE + 1; i++)
    {
        if(find_solved[i])
        {
            solved_index = i;
            ROS_DEBUG("pnp find index %d", i);
            break;
        }
    }

    for (int i = 0; i <= PNP_SIZE; i++)
    {
        Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
        
        Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        
        Vs[i] = Vector3d(para_Speed[i][0], para_Speed[i][1], para_Speed[i][2]);
        
        Bas[i] = Vector3d(para_Bias[i][0],
                          para_Bias[i][1],
                          para_Bias[i][2]);
        
        Bgs[i] = Vector3d(para_Bias[i][3],
                          para_Bias[i][4],
                          para_Bias[i][5]);
    }
    
}

void MotionOnlyEstimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }
    
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    
    if (frame_count != 0)
    {
        //covariance propagate
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        
        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);
        
        //midpoint integration
        {
            int j = frame_count;
            Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
            Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
            Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
            Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
            Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
            Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
            Vs[j] += dt * un_acc;
        }
        
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void MotionOnlyEstimator::processImage(vector<IMG_MSG_LOCAL> &feature_msg, double header)
{
    //add feature
    features[frame_count] = feature_msg;
    Headers[frame_count] = header;
    updateFeatures(feature_msg);
    
    if(frame_count < PNP_SIZE)
    {
        frame_count++;
        return;
    }
    
    solve_ceres();
    
    slideWindow();
}

void MotionOnlyEstimator::solve_ceres()
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    
    loss_function = new ceres::CauchyLoss(1.0);
    int start_index = 0;

    for (int i = start_index; i < PNP_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_Speed[i], SIZE_SPEED);
        problem.AddParameterBlock(para_Bias[i], SIZE_BIAS);
        
        if(find_solved[i])
        {
            problem.SetParameterBlockConstant(para_Pose[i]);
            problem.SetParameterBlockConstant(para_Speed[i]);
        }
        problem.SetParameterBlockConstant(para_Bias[i]);
        
    }
    
    for (int i = start_index; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    }
    
    old2new();
    //IMU factor
    for (int i = start_index; i < PNP_SIZE; i++)
    {
        int j = i + 1;
        IMUFactorPnP* imu_factor = new IMUFactorPnP(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_Speed[i], para_Bias[i], para_Pose[j], para_Speed[j], para_Bias[j]);
    }
    
    for(int i = start_index; i <= PNP_SIZE; i++)
    {
        for(auto &it : features[i])
        {
            PerspectiveFactor *f = new PerspectiveFactor(it.observation, it.position, it.track_num);
            problem.AddResidualBlock(f, loss_function, para_Pose[i], para_Ex_Pose[0]);
        }
    }
    
    ceres::Solver::Options options;
    
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 1;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.use_explicit_schur_complement = true;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 5;
    //options.use_nonmonotonic_steps = true;
    options.max_solver_time_in_seconds = 0.01;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    new2old();
    
}
/*
 marginalize the state from the sliding window and change feature start frame
 */
void MotionOnlyEstimator::slideWindow()
{
    if (frame_count == PNP_SIZE)
    {
        for (int i = 0; i < PNP_SIZE; i++)
        {
            Rs[i].swap(Rs[i + 1]);
            std::swap(pre_integrations[i], pre_integrations[i + 1]);
            dt_buf[i].swap(dt_buf[i + 1]);
            linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
            angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);
            Headers[i] = Headers[i + 1];
            Ps[i].swap(Ps[i + 1]);
            Vs[i].swap(Vs[i + 1]);
            std::swap(features[i], features[i + 1]);
            find_solved[i] = find_solved[i + 1];
            std::swap(find_solved_vins[i], find_solved_vins[i + 1]);
        }
        Headers[PNP_SIZE] = Headers[PNP_SIZE - 1];
        Ps[PNP_SIZE] = Ps[PNP_SIZE - 1];
        Vs[PNP_SIZE] = Vs[PNP_SIZE - 1];
        Rs[PNP_SIZE] = Rs[PNP_SIZE - 1];
        Bas[PNP_SIZE] = Bas[PNP_SIZE - 1];
        Bgs[PNP_SIZE] = Bgs[PNP_SIZE - 1];
        find_solved[PNP_SIZE] = false;
        if(pre_integrations[PNP_SIZE] != NULL)
        {
            delete pre_integrations[PNP_SIZE];
        }
        pre_integrations[PNP_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[PNP_SIZE], Bgs[PNP_SIZE]};
        
        features[PNP_SIZE].clear();
        dt_buf[PNP_SIZE].clear();
        linear_acceleration_buf[PNP_SIZE].clear();
        angular_velocity_buf[PNP_SIZE].clear();
        
    }
}