^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_obstacle_cruise_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.45.0 (2025-05-22)
-------------------

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* feat(path_optimizer): additional failure logging and failure mode handling (`#10276 <https://github.com/autowarefoundation/autoware_universe/issues/10276>`_)
  MRM when MPT fails
* feat!: replace VelocityLimit messages with autoware_internal_planning_msgs (`#10273 <https://github.com/autowarefoundation/autoware_universe/issues/10273>`_)
* feat: adaption to ROS nodes guidelines about directory structure (`#10268 <https://github.com/autowarefoundation/autoware_universe/issues/10268>`_)
* feat(Autoware_planning_factor_interface): replace tier4_msgs with autoware_internal_msgs (`#10204 <https://github.com/autowarefoundation/autoware_universe/issues/10204>`_)
* Contributors: Arjun Jagdish Ram, Hayato Mizushima, NorahXiong, Ryohsuke Mitsudome, Yutaka Kondo, 心刚

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* feat(autoware_vehicle_info_utils): replace autoware_universe_utils with autoware_utils (`#10167 <https://github.com/autowarefoundation/autoware_universe/issues/10167>`_)
* feat(planning_test_manager): abstract message-specific functions (`#9882 <https://github.com/autowarefoundation/autoware_universe/issues/9882>`_)
  * abstract message-specific functions
  * include necessary header
  * adapt velocity_smoother to new test manager
  * adapt behavior_velocity_planner to new test manager
  * adapt path_optimizer to new test manager
  * fix output subscription
  * adapt behavior_path_planner to new test manager
  * adapt scenario_selector to new test manager
  * adapt freespace_planner to new test manager
  * adapt planning_validator to new test manager
  * adapt obstacle_stop_planner to new test manager
  * adapt obstacle_cruise_planner to new test manager
  * disable test for freespace_planner
  * adapt behavior_velocity_crosswalk_module to new test manager
  * adapt behavior_path_lane_change_module to new test manager
  * adapt behavior_path_avoidance_by_lane_change_module to new test manager
  * adapt behavior_path_dynamic_obstacle_avoidance_module to new test manager
  * adapt behavior_path_external_request_lane_change_module to new test manager
  * adapt behavior_path_side_shift_module to new test manager
  * adapt behavior_path_static_obstacle_avoidance_module to new test manager
  * adapt path_smoother to new test manager
  * adapt behavior_velocity_blind_spot_module to new test manager
  * adapt behavior_velocity_detection_area_module to new test manager
  * adapt behavior_velocity_intersection_module to new test manager
  * adapt behavior_velocity_no_stopping_area_module to new test manager
  * adapt behavior_velocity_run_out_module to new test manager
  * adapt behavior_velocity_stop_line_module to new test manager
  * adapt behavior_velocity_traffic_light_module to new test manager
  * adapt behavior_velocity_virtual_traffic_light_module to new test manager
  * adapt behavior_velocity_walkway_module to new test manager
  * adapt motion_velocity_planner_node_universe to new test manager
  * include necessary headers
  * Odometries -> Odometry
  ---------
  Co-authored-by: Takayuki Murooka <takayuki5168@gmail.com>
* Contributors: Fumiya Watanabe, Mitsuhiro Sakamoto, Ryohsuke Mitsudome, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_obstacle_cruise_planner)!: tier4_debug_msgs changed to autoware_internal_debug_msgs in autoware_obstacle_cruise_planner (`#9905 <https://github.com/autowarefoundation/autoware_universe/issues/9905>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files planning/autoware_obstacle_cruise_planner
* chore(planning): move package directory for planning factor interface (`#9948 <https://github.com/autowarefoundation/autoware_universe/issues/9948>`_)
  * chore: add new package for planning factor interface
  * chore(surround_obstacle_checker): update include file
  * chore(obstacle_stop_planner): update include file
  * chore(obstacle_cruise_planner): update include file
  * chore(motion_velocity_planner): update include file
  * chore(bpp): update include file
  * chore(bvp-common): update include file
  * chore(blind_spot): update include file
  * chore(crosswalk): update include file
  * chore(detection_area): update include file
  * chore(intersection): update include file
  * chore(no_drivable_area): update include file
  * chore(no_stopping_area): update include file
  * chore(occlusion_spot): update include file
  * chore(run_out): update include file
  * chore(speed_bump): update include file
  * chore(stop_line): update include file
  * chore(template_module): update include file
  * chore(traffic_light): update include file
  * chore(vtl): update include file
  * chore(walkway): update include file
  * chore(motion_utils): remove factor interface
  ---------
* feat(planning_factor)!: remove velocity_factor, steering_factor and introduce planning_factor (`#9927 <https://github.com/autowarefoundation/autoware_universe/issues/9927>`_)
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
  Co-authored-by: satoshi-ota <satoshi.ota928@gmail.com>
* docs(obstacle_cruise_planner): add supplemental figures (`#9154 <https://github.com/autowarefoundation/autoware_universe/issues/9154>`_)
  * add behavior determination flowchart
  * add cruise planning block diagram
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Fumiya Watanabe, Mamoru Sobue, Mitsuhiro Sakamoto, Satoshi OTA, Vishal Chauhan

0.40.0 (2024-12-12)
-------------------
* Merge branch 'main' into release-0.40.0
* Revert "chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)"
  This reverts commit c9f0f2688c57b0f657f5c1f28f036a970682e7f5.
* refactor(obstacle_cruise_planner)!: refactor rviz and terminal info (`#9594 <https://github.com/autowarefoundation/autoware_universe/issues/9594>`_)
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)
  * chore(package.xml): bump version to 0.39.0
  * fix: fix ticket links in CHANGELOG.rst
  * fix: remove unnecessary diff
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* fix(cpplint): include what you use - planning (`#9570 <https://github.com/autowarefoundation/autoware_universe/issues/9570>`_)
* fix(obstacle_cruise_planner)!: remove stop reason (`#9464 <https://github.com/autowarefoundation/autoware_universe/issues/9464>`_)
  fix(obstacle_cruise_planner): remove stop reason
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat(obstacle_cruise_planner): outputs velocity factor when the ego follows front vehicle. (`#9359 <https://github.com/autowarefoundation/autoware_universe/issues/9359>`_)
  * feat(obstacle_cruise_planner): outputs velocity factor when the ego follows front vehicle.
  * fix: cppcheck
  ---------
* fix(autoware_obstacle_cruise_planner): fix clang-diagnostic-delete-abstract-non-virtual-dtor (`#9419 <https://github.com/autowarefoundation/autoware_universe/issues/9419>`_)
  fix: clang-diagnostic-delete-abstract-non-virtual-dtor
* fix(autoware_obstacle_cruise_planner): fix clang-diagnostic-defaulted-function-deleted (`#9398 <https://github.com/autowarefoundation/autoware_universe/issues/9398>`_)
  fix: clang-diagnostic-defaulted-function-deleted
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat(tier4_metric_msgs): apply tier4_metric_msgs for scenario_simulator_v2_adapter, control_evaluator, planning_evaluator, autonomous_emergency_braking, obstacle_cruise_planner, motion_velocity_planner, processing_time_checker (`#9180 <https://github.com/autowarefoundation/autoware_universe/issues/9180>`_)
  * first commit
  * fix building errs.
  * change diagnostic messages to metric messages for publishing decision.
  * fix bug about motion_velocity_planner
  * change the diagnostic msg to metric msg in autoware_obstacle_cruise_planner.
  * tmp save for planning_evaluator
  * change the topic to which metrics published to.
  * fix typo.
  * remove unnesessary publishing of metrics.
  * mke planning_evaluator publish msg of MetricArray instead of Diags.
  * update aeb with metric type for decision.
  * fix some bug
  * remove autoware_evaluator_utils package.
  * remove diagnostic_msgs dependency of planning_evaluator
  * use metric_msgs for autoware_processing_time_checker.
  * rewrite diagnostic_convertor to scenario_simulator_v2_adapter, supporting metric_msgs.
  * pre-commit and fix typo
  * publish metrics even if there is no metric in the MetricArray.
  * modify the metric name of processing_time.
  * update unit test for test_planning/control_evaluator
  * manual pre-commit
  ---------
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* chore(obstacle_cruise_planner): add function tests for a utils function (`#9206 <https://github.com/autowarefoundation/autoware_universe/issues/9206>`_)
  * add utils test
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kem (TiankuiXian), M. Fatih Cırıt, Ryohsuke Mitsudome, Satoshi OTA, Yuki TAKAGI, Yutaka Kondo, kobayu858

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* feat(tier4_metric_msgs): apply tier4_metric_msgs for scenario_simulator_v2_adapter, control_evaluator, planning_evaluator, autonomous_emergency_braking, obstacle_cruise_planner, motion_velocity_planner, processing_time_checker (`#9180 <https://github.com/autowarefoundation/autoware_universe/issues/9180>`_)
  * first commit
  * fix building errs.
  * change diagnostic messages to metric messages for publishing decision.
  * fix bug about motion_velocity_planner
  * change the diagnostic msg to metric msg in autoware_obstacle_cruise_planner.
  * tmp save for planning_evaluator
  * change the topic to which metrics published to.
  * fix typo.
  * remove unnesessary publishing of metrics.
  * mke planning_evaluator publish msg of MetricArray instead of Diags.
  * update aeb with metric type for decision.
  * fix some bug
  * remove autoware_evaluator_utils package.
  * remove diagnostic_msgs dependency of planning_evaluator
  * use metric_msgs for autoware_processing_time_checker.
  * rewrite diagnostic_convertor to scenario_simulator_v2_adapter, supporting metric_msgs.
  * pre-commit and fix typo
  * publish metrics even if there is no metric in the MetricArray.
  * modify the metric name of processing_time.
  * update unit test for test_planning/control_evaluator
  * manual pre-commit
  ---------
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* chore(obstacle_cruise_planner): add function tests for a utils function (`#9206 <https://github.com/autowarefoundation/autoware_universe/issues/9206>`_)
  * add utils test
  ---------
* Contributors: Esteve Fernandez, Kem (TiankuiXian), Yuki TAKAGI, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(object_recognition_utils): add autoware prefix to object_recognition_utils (`#8946 <https://github.com/autowarefoundation/autoware_universe/issues/8946>`_)
* refactor(osqp_interface): added autoware prefix to osqp_interface (`#8958 <https://github.com/autowarefoundation/autoware_universe/issues/8958>`_)
* chore(obstacle_cruise_planner): add maintainer (`#9077 <https://github.com/autowarefoundation/autoware_universe/issues/9077>`_)
* feat(obstacle_cruise_planner): improve stop and cruise behavior for cut-in & out (`#8072 <https://github.com/autowarefoundation/autoware_universe/issues/8072>`_)
  * feat(obstacle_cruise_planner): improve stop and cruise behavior for cut-in & out
  * cleanup, add stop safety margin for transient objects
  style(pre-commit): autofix
  * fix: debug
  * fix: precommit error
  * fix: unused-variable
  * feat: improve cruise behavior for outside obstacles
  * fix projected velocity, improve transient obstacle behavior
  * feat: add predefined deceleration rate for VRUs
  * feat: update
  ---------
* refactor(autoware_interpolation): prefix package and namespace with autoware (`#8088 <https://github.com/autowarefoundation/autoware_universe/issues/8088>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* refactor(signal_processing): prefix package and namespace with autoware (`#8541 <https://github.com/autowarefoundation/autoware_universe/issues/8541>`_)
* fix(motion_planning): align the parameters with launcher (`#8792 <https://github.com/autowarefoundation/autoware_universe/issues/8792>`_)
  parameters in motion_planning aligned
* fix(velocity_smoother, obstacle_cruise_planner ): float type of processing time was wrong (`#8161 <https://github.com/autowarefoundation/autoware_universe/issues/8161>`_)
  fix(velocity_smoother): float type of processing time was wrong
* feat(cruise_planner,planning_evaluator): add cruise and slow down diags (`#7960 <https://github.com/autowarefoundation/autoware_universe/issues/7960>`_)
  * add cruise and slow down diags to cruise planner
  * add cruise types
  * adjust planning eval
  ---------
* feat(obstacle_cruise_planner): prevent chattering when using point cloud (`#7861 <https://github.com/autowarefoundation/autoware_universe/issues/7861>`_)
  * prevent chattering of stop planning
  * Update planning/autoware_obstacle_cruise_planner/src/node.cpp
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  * fix stop position oscillation
  ---------
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* feat(obstacle_cruise_planner): add diagnostics publishing to cruise planner (`#7836 <https://github.com/autowarefoundation/autoware_universe/issues/7836>`_)
  add diagnostics publishing to cruise planner
* feat(obstacle_cruise_planner): support pointcloud-based obstacles (`#6907 <https://github.com/autowarefoundation/autoware_universe/issues/6907>`_)
  * add pointcloud to obstacle properties
  * add tf listener & pointcloud subscriber
  * add parameters for pointcloud obstacle
  * add type aliases
  * convert pointcloud to obstacle
  * add type alias
  * add polygon conversion for pointcloud obstacle
  * initialize twist & pose of pointcloud obstacle
  * overload to handle both obstacle & predicted path
  * implement ego behavior determination against pointcloud obstacles
  * generate obstacle from point
  * revert getCollisionIndex()
  * generate obstacle from each point in cloud
  * set pointcloud obstacle velocity to 0
  * use tf buffer & listener with pointers
  * update latest pointcloud data
  * add topic remap
  * remove unnecessary includes
  * set slow down obstacle velocity to 0
  * add flag to consider pointcloud obstacle for stopping & slowing down
  * style(pre-commit): autofix
  * downsample pointcloud using voxel grid
  * change  shape type of pointcloud obstacle to polygon
  * convert pointcloud to obstacle by clustering
  * add parameters for clustering
  * add max_num_points parameter to dummy object
  * downsample pointcloud when the number of points is larger than max_num_points
  * add max_num_points property to dummy bus
  * add parameters for pointcloud based obstacles
  * store pointcloud in obstacle struct
  * change obstacle conversion method
  * migrate previous changes to new package
  * store necessary points only
  * move use_pointcloud to common parameter
  * extract necessary points from pointcloud
  * add use_pointcloud parameter to planner interface
  * fix obstacle conversion
  * fix collision point determination
  * simplify pointcloud transformation
  * style(pre-commit): autofix
  * fix collision point determination
  * pick nearest stop collision point
  * check collision for every point in cluster
  * migrate previous changes to new files
  * reduce diff
  * remove use_pointcloud parameter
  * add parameters for pointcloud filtering
  * add autoware namespace
  * Revert "add max_num_points parameter to dummy object"
  This reverts commit 98bcd0856f861d23c9f7989d8128939ec0b3e27c.
  * Revert "downsample pointcloud when the number of points is larger than max_num_points"
  This reverts commit fb00b59d8f14cec6810e7fab12bc34d8a0c617c7.
  * Revert "add max_num_points property to dummy bus"
  This reverts commit 5f9e4ab5ae7d8d46521c736b1d259040121f3bc5.
  * feat(diagnostic_graph_utils): add logging tool
  * fix all OK
  * feat(default_ad_api): add log when operation mode change fails
  * get only the necessary one of object or pointcloud data
  * addfield for obstacle source type
  * enable simultaneous use of PredictedObjects and PointCloud
  * separate convertToObstacles() by source type
  * avoid using pointer
  * reduce diff
  * make nest shallower
  * define vector concatenate function
  * shorten variable names
  * fix redundant condition
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Takagi, Isamu <isamu.takagi@tier4.jp>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
* feat: add `autoware\_` prefix to `lanelet2_extension` (`#7640 <https://github.com/autowarefoundation/autoware_universe/issues/7640>`_)
* fix(autoware_obstacle_cruise_planner): fix shadowVariable warning in generateSlowDownTrajectory (`#7659 <https://github.com/autowarefoundation/autoware_universe/issues/7659>`_)
* fix(autoware_obstacle_cruise_planner): fix shadowVariable warning (`#7656 <https://github.com/autowarefoundation/autoware_universe/issues/7656>`_)
  * fix(autoware_obstacle_cruise_planner): fix shadowVariable warning
  * fix
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_obstacle_cruise_planner): fix knownConditionTrueFalse warnings (`#7620 <https://github.com/autowarefoundation/autoware_universe/issues/7620>`_)
  * fix(autoware_obstacle_cruise_planner): fix knownConditionTrueFalse warnings
  * fix
  ---------
* fix(autoware_obstacle_cruise_planner): fix unreadVariable warning (`#7627 <https://github.com/autowarefoundation/autoware_universe/issues/7627>`_)
* refactor(obstacle_cruise_planner): apply clang-tidy check (`#7553 <https://github.com/autowarefoundation/autoware_universe/issues/7553>`_)
  obstacle_cruise
* refactor(universe_utils/motion_utils)!: add autoware namespace (`#7594 <https://github.com/autowarefoundation/autoware_universe/issues/7594>`_)
* fix(autoware_obstacle_cruise_planner): fix assignBoolToFloat warning (`#7541 <https://github.com/autowarefoundation/autoware_universe/issues/7541>`_)
  * fix(autoware_obstacle_cruise_planner): fix assignBoolToFloat warning
  * delete unnecessary file
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_obstacle_cruise_planner): fix unusedScopedObject bug (`#7569 <https://github.com/autowarefoundation/autoware_universe/issues/7569>`_)
* refactor(motion_utils)!: add autoware prefix and include dir (`#7539 <https://github.com/autowarefoundation/autoware_universe/issues/7539>`_)
  refactor(motion_utils): add autoware prefix and include dir
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* feat(obstacle_cruise_planner): rename to include/autoware/{package_name} (`#7510 <https://github.com/autowarefoundation/autoware_universe/issues/7510>`_)
* refactor(test_utils): move to common folder (`#7158 <https://github.com/autowarefoundation/autoware_universe/issues/7158>`_)
  * Move autoware planning test manager to autoware namespace
  * fix package share directory for behavior path planner
  * renaming files and directory
  * rename variables that has planning_test_utils in its name.
  * use autoware namespace for test utils
  * move folder to common
  * update .pages file
  * fix test error
  * removed obstacle velocity limiter test artifact
  * remove namespace from planning validator, it has using keyword
  ---------
* refactor(obstacle_cruise_planner)!: add autoware\_ prefix (`#7419 <https://github.com/autowarefoundation/autoware_universe/issues/7419>`_)
* Contributors: Berkay Karaman, Esteve Fernandez, Koichi98, Kosuke Takeuchi, Mamoru Sobue, Mitsuhiro Sakamoto, Ryuta Kambe, Takayuki Murooka, Yuki TAKAGI, Yutaka Kondo, Zhe Shen, Zulfaqar Azmi, danielsanchezaran

0.26.0 (2024-04-03)
-------------------
