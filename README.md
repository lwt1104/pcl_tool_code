 ./correspondence_grouping ../cup_model0.pcd  -r --model_ss 5 --scene_ss 15 --rf_rad 20 --descr_rad 25 --cg_size 38 -ac -c  frame-5_seg.pcd 

./correspondence_grouping ../cup_model0.pcd -r --model_ss 7.5 --scene_ss 20 --rf_rad 15 --descr_rad 25 --cg_size 38 -ac -c frame-0.pcd 

./global_hypothesis_verification --model_ss 0.02 --scene_ss 0.02 --rf_rad 0.015 --descr_rad 0.03 --cg_size 0.05 ../cup_model0.pcd ../cup_scene_3.pcd 

// Note don't name a folder containing vfh
./build_tree ../data/

./nearest_neighbors ../cup_model1.pcd -k 1 -thresh 50

./plane_cluster_seg frame-1.pcd -min_depth 0.3 -max_depth 2.0
