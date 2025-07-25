^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_simple_object_merger
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* feat(radar): update radar pipeline (`#10580 <https://github.com/autowarefoundation/autoware_universe/issues/10580>`_)
  * fix(tier4_perception_launch): update radar filter launch configuration for improved object handling
  * fix(autoware_simple_object_merger): change QoS settings to best effort for input subscriptions
  * fix(autoware_simple_object_merger): change publisher QoS to reliable for output objects
  * fix(tier4_perception_launch): remove commented-out radar filter pipeline from launch configuration
  * style(pre-commit): autofix
  * fix(tier4_perception_launch): remove unnecessary radar filter dependencies from package.xml
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_simple_object_merger): created Schema file and updated ReadME file for parameters setting (`#9993 <https://github.com/autowarefoundation/autoware_universe/issues/9993>`_)
  * feat(autoware_simple_object_merger): Created Schema file and updated ReadME file for parameters setting
  * style(pre-commit): autofix
  * Update README.md
  updated readme file
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* Contributors: Taekjin LEE, TaikiYamada4, Vishal Chauhan

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(simple_object_merger): add approximatefilter (`#10462 <https://github.com/autowarefoundation/autoware_universe/issues/10462>`_)
  * fix(simple_object_merger): add approximatefilter
  * Update perception/autoware_simple_object_merger/src/simple_object_merger_node.cpp
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * Update perception/autoware_simple_object_merger/src/simple_object_merger_node.cpp
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * Update perception/autoware_simple_object_merger/src/simple_object_merger_node.cpp
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * Update perception/autoware_simple_object_merger/src/simple_object_merger_node.cpp
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  * Update perception/autoware_simple_object_merger/src/simple_object_merger_node.hpp
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
  ---------
  Co-authored-by: Taekjin LEE <technolojin@gmail.com>
* feat: reduce warn log frequency (`#10414 <https://github.com/autowarefoundation/autoware_universe/issues/10414>`_)
  * feat: limit warning message frequency to 3.0 sec
  * refactor: use THROTTLE ERROR macro
  fix: reduce code complexity
  ---------
  Co-authored-by: Jian Kang <jian.kang@tier4.jp>
* chore(perception): code owner revision (`#10358 <https://github.com/autowarefoundation/autoware_universe/issues/10358>`_)
  * feat: add Masato Saeki and Taekjin Lee as maintainer to multiple package.xml files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Kang, Ryohsuke Mitsudome, Taekjin LEE, badai nguyen

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* Contributors: Hayato Mizushima, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* chore: refine maintainer list (`#10110 <https://github.com/autowarefoundation/autoware_universe/issues/10110>`_)
  * chore: remove Miura from maintainer
  * chore: add Taekjin-san to perception_utils package maintainer
  ---------
* Contributors: Fumiya Watanabe, Shunsuke Miura, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------

0.40.0 (2024-12-12)
-------------------
* Revert "chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)"
  This reverts commit c9f0f2688c57b0f657f5c1f28f036a970682e7f5.
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* chore(package.xml): bump version to 0.39.0 (`#9587 <https://github.com/autowarefoundation/autoware_universe/issues/9587>`_)
  * chore(package.xml): bump version to 0.39.0
  * fix: fix ticket links in CHANGELOG.rst
  * fix: remove unnecessary diff
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* fix: fix ticket links in CHANGELOG.rst (`#9588 <https://github.com/autowarefoundation/autoware_universe/issues/9588>`_)
* 0.39.0
* update changelog
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, Ryohsuke Mitsudome, Yutaka Kondo

0.39.0 (2024-11-25)
-------------------
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Yutaka Kondo

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(radar_tracks_msgs_converter, simple_object_merger, radar_tracks_noise_filter)!: add package name prefix of autoware\_ (`#8173 <https://github.com/autowarefoundation/autoware_universe/issues/8173>`_)
  * refactor: rename radar_tracks_msgs_converter package to autoware_radar_tracks_msgs_converter
  * refactor: rename simple_object_merger package to autoware_simple_object_merger
  * refactor: rename sensing/radar_tracks_noise_filter to sensing/autoware_radar_tracks_noise_filter
  ---------
* Contributors: Taekjin LEE, Yutaka Kondo

0.26.0 (2024-04-03)
-------------------
