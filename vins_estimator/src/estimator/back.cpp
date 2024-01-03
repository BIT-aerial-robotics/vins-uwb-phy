ROS_INFO("begin optimizationwithLine ");
// ROS_INFO("feature line size is %d ",f_manager.getLineFeatureCount());
ceres::Problem problem;
ceres::LossFunction *loss_function;
loss_function = new ceres::HuberLoss(1.0);
vector2double();
if (1) // vins 原有
{
    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        if (USE_IMU)
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    if (!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            // ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else
        {
            // ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }
    problem.AddParameterBlock(para_Td[0], 1);

    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);

    ROS_INFO("add vins orinal pose ParameterBlock ");
    cout << "in add process address: " << last_marginalization_info << endl;
    cout << "in add process size === " << last_marginalization_parameter_blocks.size() << endl;
    // cout<<" valid:"<<last_marginalization_info->valid<<endl;
    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        cout << "in last_marginalization_info process size === " << last_marginalization_parameter_blocks.size() << endl;
        for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            cout << "idx: " << i << " address :" << last_marginalization_parameter_blocks[i] << " ";
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    ROS_INFO("add vins orinal marginalizationFactor ");
    if (USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;
            IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }
    ROS_INFO("add vins orinal residual imu and ");
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }

            if (STEREO && it_per_frame.is_stereo)
            {
                Vector3d pts_j_right = it_per_frame.pointRight;
                if (imu_i != imu_j)
                {
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                           it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
                else
                {
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                           it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
            }
            f_m_cnt++;
        }
    }
    ROS_INFO("visual measurement count: %d", f_m_cnt);
}
ROS_INFO("add vins orinal residual finish");
if (0) // Line feature
{
    // Line feature
    int line_m_cnt = 0;
    int linefeature_index = -1;
    for (auto &it_per_id : f_manager.feature_line)
    {
        it_per_id.used_num = it_per_id.feature_line_per_frame.size();                                                       // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation)) // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
            continue;
        ++linefeature_index; // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式
        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();
        problem.AddParameterBlock(para_LineFeature[linefeature_index], SIZE_LINE, local_parameterization_line); // p,q
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        for (auto &it_per_frame : it_per_id.feature_line_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                // continue;
            }
            Eigen::Vector4d obs = it_per_frame.lineobs;              // 在第j帧图像上的观测
            lineProjectionFactor *f = new lineProjectionFactor(obs); // 特征重投影误差
            problem.AddResidualBlock(f, loss_function,
                                     para_Pose[imu_j],
                                     para_Ex_Pose[0],
                                     para_LineFeature[linefeature_index]);
            line_m_cnt++;
        }
    }
    ROS_INFO("lineFactor: %d", line_m_cnt);
}
ceres::Solver::Options options;
options.linear_solver_type = ceres::DENSE_SCHUR;
// options.num_threads = 2;
options.trust_region_strategy_type = ceres::DOGLEG;
options.max_num_iterations = NUM_ITERATIONS;
ceres::Solver::Summary summary;
ceres::Solve(options, &problem, &summary);
double2vector2(); // Line pose change
std::cout << "optimizationwithLine " << summary.BriefReport() << std::endl;
// f_manager.removeLineOutlier(Rs,Ps,tic,ric);
if (marginalization_flag == MARGIN_OLD)
{
    // 构建一个新的 prior info
    MarginalizationInfo *marginalization_info = new MarginalizationInfo();
    vector2double();
    printf("MARGIN_OLD add MarginalizationInfo\n");
    if (last_marginalization_info && last_marginalization_info->valid)
    {
        vector<int> drop_set;
        printf("%d --  ", static_cast<int>(last_marginalization_parameter_blocks.size()));
        for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
        {
            if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                drop_set.push_back(i);
        }
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                       last_marginalization_parameter_blocks,
                                                                       drop_set);
        marginalization_info->addResidualBlockInfo(residual_block_info);
    }

    if (USE_IMU)
    {
        if (pre_integrations[1]->sum_dt < 10.0)
        {
            IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }
    }

    {
        int feature_index = -1;
        for (auto &it_per_id : f_manager.feature)
        {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (it_per_id.used_num < 4)
                continue;

            ++feature_index;

            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            if (imu_i != 0)
                continue;

            Vector3d pts_i = it_per_id.feature_per_frame[0].point;

            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                imu_j++;
                if (imu_i != imu_j)
                {
                    Vector3d pts_j = it_per_frame.point;
                    ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                              it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                   vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                   vector<int>{0, 3});
                    // marginalization_info->addResidualBlockInfo(residual_block_info);
                }
                if (STEREO && it_per_frame.is_stereo)
                {
                    Vector3d pts_j_right = it_per_frame.pointRight;
                    if (imu_i != imu_j)
                    {
                        ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                               it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                       vector<int>{0, 4});
                        // marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    else
                    {
                        ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                               it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                       vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                       vector<int>{2});
                        // marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }
            }
        }
    }
    if(0)
    {
        // Line feature
        int linefeature_index = -1;
        for (auto &it_per_id : f_manager.feature_line)
        {
            it_per_id.used_num = it_per_id.feature_line_per_frame.size();                                                       // 已经被多少帧观测到， 这个已经在三角化那个函数里说了
            if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.is_triangulation)) // 如果这个特征才被观测到，那就跳过。实际上这里为啥不直接用如果特征没有三角化这个条件。
                continue;
            ++linefeature_index;
            // 这个变量会记录feature在 para_Feature 里的位置， 将深度存入para_Feature时索引的记录也是用的这种方式

            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            if (imu_i != 0) // 如果这个特征的初始帧 不对应 要marg掉的最老帧0, 那就不用marg这个特征。即marg掉帧的时候，我们marg掉这帧上三角化的那些点
                continue;

            for (auto &it_per_frame : it_per_id.feature_line_per_frame)
            {
                imu_j++;

                std::vector<int> drop_set;
                if (imu_i == imu_j)
                { // drop_set = vector<int>{0, 2};   // marg pose and feature,  !!!! do not need marg, just drop they  !!!
                    continue;
                }
                else
                {
                    drop_set = vector<int>{2}; // marg feature
                }

                Eigen::Vector4d obs = it_per_frame.lineobs;              // 在第j帧图像上的观测
                lineProjectionFactor *f = new lineProjectionFactor(obs); // 特征重投影误差

                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                               vector<double *>{para_Pose[imu_j], para_Ex_Pose[0], para_LineFeature[linefeature_index]},
                                                                               drop_set); // vector<int>{0, 2} 表示要marg的参数下标，比如这里对应para_Pose[imu_i], para_Feature[feature_index]
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }
    }
    TicToc t_pre_margin;
    marginalization_info->preMarginalize();
    ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

    TicToc t_margin;
    marginalization_info->marginalize();
    ROS_DEBUG("marginalization %f ms", t_margin.toc());

    std::unordered_map<long, double *> addr_shift;
    for (int i = 1; i <= WINDOW_SIZE; i++)
    {
        addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1]; //
        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
        addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
    vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

    if (last_marginalization_info)
        delete last_marginalization_info;
    last_marginalization_info = marginalization_info;
    last_marginalization_parameter_blocks = parameter_blocks;
    cout << "address=" << last_marginalization_info << " " << parameter_blocks.size() << "  " << last_marginalization_parameter_blocks.size() << endl;
}
else
{

    printf("MARGIN_Second New add MarginalizationInfo\n");
    if (last_marginalization_info &&
        std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
    {

        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();
        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);

            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        TicToc t_pre_margin;
        ROS_DEBUG("begin marginalization");
        marginalization_info->preMarginalize();
        ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

        TicToc t_margin;
        ROS_DEBUG("begin marginalization");
        marginalization_info->marginalize();
        ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            if (i == WINDOW_SIZE - 1)
                continue;
            else if (i == WINDOW_SIZE)
            {
                addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                if (USE_IMU)
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
            }
            else
            {
                addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                if (USE_IMU)
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
            }
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
}