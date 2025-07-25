^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_bluetooth_monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Contributors: Hayato Mizushima, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat: apply `autoware\_` prefix for `bluetooth_monitor` (`#9960 <https://github.com/autowarefoundation/autoware_universe/issues/9960>`_)
  * feat(bluetooth_monitor): apply `autoware\_` prefix (see below):
  * In this commit, I did not organize a folder structure.
  The folder structure will be organized in the next some commits.
  * The changes will follow the Autoware's guideline as below:
  - https://autowarefoundation.github.io/autoware-documentation/main/contributing/coding-guidelines/ros-nodes/directory-structure/#package-folder
  * rename(bluetooth_monitor): move headers under `include/autoware`:
  * Fixes due to this changes for .hpp/.cpp files will be applied in the next commit
  * fix(bluetooth_monitor): fix include paths
  * To follow the previous commit
  * bug(bluetooth_monitor): fix a missing prefix bug
  * rename: `bluetooth_monitor` => `autoware_bluetooth_monitor`
  * style(pre-commit): autofix
  * bug(autoware_bluetooth_monitor): revert wrongly updated copyrights
  * bug(autoware_bluetooth_monitor): `autoware\_` prefix is not needed here
  * update: `CODEOWNERS`
  * update(autoware_bluetooth_monitor): `README.md`
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Fumiya Watanabe, Junya Sasaki

0.40.0 (2024-12-12)
-------------------
* Merge branch 'main' into release-0.40.0
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
* fix(cpplint): include what you use - system (`#9573 <https://github.com/autowarefoundation/autoware_universe/issues/9573>`_)
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
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo

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
* fix(docs): fix file name for bluetooth monitor schema (`#8308 <https://github.com/autowarefoundation/autoware_universe/issues/8308>`_)
  * fix file name for schema
  * the variable name should be addresses instead
  ---------
* fix(bluetooth_monitor): fix unreadVariable (`#8371 <https://github.com/autowarefoundation/autoware_universe/issues/8371>`_)
  fix:unreadVariable
* fix(bluetooth_monitor): apply cppcheck-suppress for cstyleCast (`#7869 <https://github.com/autowarefoundation/autoware_universe/issues/7869>`_)
* Contributors: Koichi98, Yutaka Kondo, Yuxuan Liu, kobayu858

0.26.0 (2024-04-03)
-------------------
* refactor(bluetooth_monitor): rework parameters (`#5239 <https://github.com/autowarefoundation/autoware_universe/issues/5239>`_)
  * refactor(bluetooth_monitor): rework parameters
  * style(pre-commit): autofix
  * doc(bluetooth_monitor): fix default for integer parameters
  * doc(bluetooth_monitor): add default for parameter addresses
  * style(pre-commit): autofix
  * doc(bluetooth_monitor): fix parameter description
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* build: mark autoware_cmake as <buildtool_depend> (`#3616 <https://github.com/autowarefoundation/autoware_universe/issues/3616>`_)
  * build: mark autoware_cmake as <buildtool_depend>
  with <build_depend>, autoware_cmake is automatically exported with ament_target_dependencies() (unecessary)
  * style(pre-commit): autofix
  * chore: fix pre-commit errors
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
* chore: sync files (`#3227 <https://github.com/autowarefoundation/autoware_universe/issues/3227>`_)
  * chore: sync files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: kenji-miyake <kenji-miyake@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* build(bluetooth_monitor): add build dependency (`#2738 <https://github.com/autowarefoundation/autoware_universe/issues/2738>`_)
* ci(pre-commit): format SVG files (`#2172 <https://github.com/autowarefoundation/autoware_universe/issues/2172>`_)
  * ci(pre-commit): format SVG files
  * ci(pre-commit): autofix
  * apply pre-commit
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(bluetooth_monitor): run bluetooth monitor with new parameter (`#1111 <https://github.com/autowarefoundation/autoware_universe/issues/1111>`_)
  * feat(bluetooth_monitor): run bluetooth monitor with new parameter
  * ci(pre-commit): autofix
  * Fixed a build error in humble
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(bluetooth_monitor): add functionality to monitor Bluetooth connection (`#862 <https://github.com/autowarefoundation/autoware_universe/issues/862>`_)
  * feat(bluetooth_monitor): add functionality to monitor Bluetooth connection
  * ci(pre-commit): autofix
  * Fixed a typo
  * Add a dependency
  * Fixed pre-commit errors
  * ci(pre-commit): autofix
  * Fixed pre-commit errors
  * Fixed uncrustify errors
  * ci(pre-commit): autofix
  * use autoware_cmake
  * Fixed license, Fixed CMakeLists.txt, and Use register_node_macro
  * Fixed license
  * Fixed link title
  * changed the way to run l2ping
  * ci(pre-commit): autofix
  * fixed clang tidy error and  removed unnecessary dependencies in CMakeLists.txt
  * corrected dependency in package.xml
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Esteve Fernandez, Kenji Miyake, Vincent Richard, Yuqi Huai, awf-autoware-bot[bot], ito-san
