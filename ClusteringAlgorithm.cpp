#include "ClusteringAlgorithm.h"



ClusteringAlgorithm::ClusteringAlgorithm() {}


void ClusteringAlgorithm::Solve(int K, std::vector<kPoint>* all_points, std::vector<std::vector<kPoint>>* clustered_points, Pathfinder* pathfinder, int max_iterations) {
	if (DEBUG_HL_KMEANS) {
		printf("Running k-means clustering for k = %d, # points = %ld\n", K, all_points->size());
	}

	int k = K;

	if (k > boost::numeric_cast<int>(all_points->size())) {
		k = boost::numeric_cast<int>(all_points->size());
	}

	std::vector<kPoint> centroids;
	std::vector<int> pointsAvailable;
	for (int i = 0; i < boost::numeric_cast<int>(all_points->size()); i++) {
		pointsAvailable.push_back(i);
	}

	for (int i = 0; i < k; i++) {
		if (pointsAvailable.empty()) {
			// 如果没有更多可用点，就不要再创建中心点了
			k = i; // 更新k的实际值
			break;
		}
		int rnd_index = rand() % pointsAvailable.size();
		int node_id = pointsAvailable.at(rnd_index);
		kPoint cPoint(-1, i, all_points->at(node_id).X, all_points->at(node_id).Y, all_points->at(node_id).Z);
		centroids.push_back(cPoint);
		if (DEBUG_HL_KMEANS) {
			printf(" New cluster at (%f, %f, %f) (at node %d)\n", cPoint.X, cPoint.Y, cPoint.Z, node_id);
		}
		pointsAvailable.erase(pointsAvailable.begin() + rnd_index);
	}

	bool cent_moved = true;
	for (int i = 0; i < max_iterations && cent_moved; ++i) {
		if (DEBUG_HL_KMEANS) {
			printf(" New iteration %d\n", i);
		}
		cent_moved = false;
		for (std::vector<kPoint>::iterator pnt_it = all_points->begin(); pnt_it != all_points->end(); ++pnt_it) {
			double min_dist = std::numeric_limits<double>::max();
			int best_ctr = pnt_it->centroid_ID;
			for (std::vector<kPoint>::iterator c = centroids.begin(); c != centroids.end(); ++c) {
				double dist_to_c = pathfinder->get_path_distance({ pnt_it->X, pnt_it->Y }, { c->X, c->Y });

				// *** MODIFICATION HERE: Check for INF distance ***
				if (dist_to_c >= INF) {
					continue; // 如果不可达，就跳过这个中心点
				}

				if (dist_to_c < min_dist) {
					min_dist = dist_to_c;
					best_ctr = c->centroid_ID;
				}
			}

			// *** MODIFICATION HERE: Check if a valid cluster was found ***
			// 如果一个点到所有中心点都不可达，min_dist会保持max(), best_ctr保持不变
			if (min_dist < std::numeric_limits<double>::max() && best_ctr != pnt_it->centroid_ID) {
				if (DEBUG_HL_KMEANS) {
					printf("  Point %d moves %d -> %d\n", pnt_it->point_ID, pnt_it->centroid_ID, best_ctr);
				}
				pnt_it->centroid_ID = best_ctr;
				cent_moved = true;
			}
		}

		if (cent_moved) {
			if (DEBUG_HL_KMEANS) {
				printf("  Updating centroids\n");
			}

			std::vector<double> sum_x;
			std::vector<double> sum_y;
			std::vector<double> sum_z;
			std::vector<int> point_count;

			for (int j = 0; j < k; j++) {
				sum_x.push_back(0.0);
				sum_y.push_back(0.0);
				sum_z.push_back(0.0);
				point_count.push_back(0);
			}

			for (std::vector<kPoint>::iterator pnt_it = all_points->begin(); pnt_it != all_points->end(); pnt_it++) {
				// *** MODIFICATION HERE: Ensure centroid_ID is valid ***
				if (pnt_it->centroid_ID >= 0 && pnt_it->centroid_ID < k) {
					int cntrd_i = pnt_it->centroid_ID;
					sum_x[cntrd_i] += pnt_it->X;
					sum_y[cntrd_i] += pnt_it->Y;
					sum_z[cntrd_i] += pnt_it->Z;
					point_count[cntrd_i]++;
				}
			}

			for (int j = 0; j < k; j++) {
				if (point_count[j] == 0) {
					if (DEBUG_HL_KMEANS) {
						printf("  ** Centroid %d has no points!\n", j);
					}
					// 空簇的处理：暂时简单地保持其原位，避免从一个可能已被分配的点中选择
					// 未来可以设计更复杂的逻辑，比如从最远的未分配点中选择
				}
				else {
					centroids.at(j).X = sum_x[j] / point_count[j];
					centroids.at(j).Y = sum_y[j] / point_count[j];
					centroids.at(j).Z = sum_z[j] / point_count[j];
				}

				if (DEBUG_HL_KMEANS) {
					printf("   Centroid %d has %d points, at (%f, %f, %f)\n", j, point_count[j], centroids.at(j).X, centroids.at(j).Y, centroids.at(j).Z);
				}
			}
		}
		else {
			if (DEBUG_HL_KMEANS) {
				printf(" Converged!\n Final solution:\n");
			}
		}
	}

	clustered_points->clear();
	for (int i = 0; i < K; i++) {
		std::vector<kPoint> clstr_i;
		clustered_points->push_back(clstr_i);
	}

	for (std::vector<kPoint>::iterator pnt_it = all_points->begin(); pnt_it != all_points->end(); pnt_it++) {
		if (pnt_it->centroid_ID >= 0 && pnt_it->centroid_ID < K) {
			clustered_points->at(pnt_it->centroid_ID).push_back(*pnt_it);
		}
		if (DEBUG_HL_KMEANS) {
			printf("  %d : %d\n", pnt_it->point_ID, pnt_it->centroid_ID);
		}
	}
}
