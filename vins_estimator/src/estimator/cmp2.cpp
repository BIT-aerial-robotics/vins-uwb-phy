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
            marginalization_info->addResidualBlockInfo(residual_block_info);
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
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
            else
            {
                ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                       it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                               vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                               vector<int>{2});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }
    }
}