^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_localization_evaluator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.45.0 (2025-05-22)
-------------------

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore(localization, perception): remove koji minoda as maintainer from multiple packages (`#10359 <https://github.com/autowarefoundation/autoware_universe/issues/10359>`_)
  fix: remove Koji Minoda as maintainer from multiple package.xml files
* Contributors: Ryohsuke Mitsudome, Taekjin LEE

0.43.0 (2025-03-21)
-------------------
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore: rename from `autoware.universe` to `autoware_universe` (`#10306 <https://github.com/autowarefoundation/autoware_universe/issues/10306>`_)
* Contributors: Hayato Mizushima, Yutaka Kondo

0.42.0 (2025-03-03)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(autoware_utils): replace autoware_universe_utils with autoware_utils  (`#10191 <https://github.com/autowarefoundation/autoware_universe/issues/10191>`_)
* Contributors: Fumiya Watanabe, 心刚

0.41.2 (2025-02-19)
-------------------
* chore: bump version to 0.41.1 (`#10088 <https://github.com/autowarefoundation/autoware_universe/issues/10088>`_)
* Contributors: Ryohsuke Mitsudome

0.41.1 (2025-02-10)
-------------------

0.41.0 (2025-01-29)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat: apply `autoware\_` prefix for `evaluator/localization_evaluator` (`#9954 <https://github.com/autowarefoundation/autoware_universe/issues/9954>`_)
  * feat(localization_evaluator): apply `autoware\_` prefix (see below):
  * In this commit, I did not organize a folder structure.
  The folder structure will be organized in the next some commits.
  * The changes will follow the Autoware's guideline as below:
  - https://autowarefoundation.github.io/autoware-documentation/main/contributing/coding-guidelines/ros-nodes/directory-structure/#package-folder
  * rename(localization_evaluator): move headers under `include/autoware`:
  * Fixes due to this changes for .hpp/.cpp files will be applied in the next commit
  * fix(localization_evaluator): fix include paths
  * To follow the previous commit
  * rename: `localization_evaluator` => `autoware_localization_evaluator`
  * style(pre-commit): autofix
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
* fix(cpplint): include what you use - evaluator (`#9566 <https://github.com/autowarefoundation/autoware_universe/issues/9566>`_)
* chore(localization_evaluator): Add some maintainers (`#9503 <https://github.com/autowarefoundation/autoware_universe/issues/9503>`_)
* refactor(evaluators, autoware_universe_utils): rename Stat class to Accumulator and move it to autoware_universe_utils (`#9459 <https://github.com/autowarefoundation/autoware_universe/issues/9459>`_)
  * add Accumulator class to autoware_universe_utils
  * use Accumulator on all evaluators.
  * pre-commit
  * found and fixed a bug. add more tests.
  * pre-commit
  * Update common/autoware_universe_utils/include/autoware/universe_utils/math/accumulator.hpp
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  ---------
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
* 0.39.0
* update changelog
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix(evaluator): missing dependency in evaluator components (`#9074 <https://github.com/autowarefoundation/autoware_universe/issues/9074>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* Contributors: Esteve Fernandez, Fumiya Watanabe, Kem (TiankuiXian), M. Fatih Cırıt, Motz, Ryohsuke Mitsudome, Yutaka Kondo, ぐるぐる

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
* refactor(evaluator/localization_evaluator): rework parameters  (`#6744 <https://github.com/autowarefoundation/autoware_universe/issues/6744>`_)
  * add param file and schema
  * style(pre-commit): autofix
  * .
  * .
  * .
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_universe_utils)!: rename from tier4_autoware_utils (`#7538 <https://github.com/autowarefoundation/autoware_universe/issues/7538>`_)
  Co-authored-by: kosuke55 <kosuke.tnp@gmail.com>
* ci(pre-commit): autoupdate (`#7499 <https://github.com/autowarefoundation/autoware_universe/issues/7499>`_)
  Co-authored-by: M. Fatih Cırıt <mfc@leodrive.ai>
* feat!: replace autoware_auto_msgs with autoware_msgs for evaluator modules (`#7241 <https://github.com/autowarefoundation/autoware_universe/issues/7241>`_)
  Co-authored-by: Cynthia Liu <cynthia.liu@autocore.ai>
  Co-authored-by: NorahXiong <norah.xiong@autocore.ai>
  Co-authored-by: beginningfan <beginning.fan@autocore.ai>
* Contributors: Ryohsuke Mitsudome, Takayuki Murooka, Yutaka Kondo, awf-autoware-bot[bot], oguzkaganozt

0.26.0 (2024-04-03)
-------------------
* chore(build): remove tier4_autoware_utils.hpp evaluator/ simulator/ (`#4839 <https://github.com/autowarefoundation/autoware_universe/issues/4839>`_)
* chore: add maintainer (`#4234 <https://github.com/autowarefoundation/autoware_universe/issues/4234>`_)
  * chore: add maintainer
  * Update evaluator/localization_evaluator/package.xml
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
  ---------
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
* style: fix typos (`#3617 <https://github.com/autowarefoundation/autoware_universe/issues/3617>`_)
  * style: fix typos in documents
  * style: fix typos in package.xml
  * style: fix typos in launch files
  * style: fix typos in comments
  ---------
* build: mark autoware_cmake as <buildtool_depend> (`#3616 <https://github.com/autowarefoundation/autoware_universe/issues/3616>`_)
  * build: mark autoware_cmake as <buildtool_depend>
  with <build_depend>, autoware_cmake is automatically exported with ament_target_dependencies() (unecessary)
  * style(pre-commit): autofix
  * chore: fix pre-commit errors
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
* feat(metrics_calculation): add kinematic and localization evaluators with metrics (`#928 <https://github.com/autowarefoundation/autoware_universe/issues/928>`_)
  * initial skeleton of localization evaluator
  * Add simple localization evaliuation framework
  * Clean and add tests
  * ci(pre-commit): autofix
  * Clean localization evaluator
  * Update kinematic evaluator
  * ci(pre-commit): autofix
  * dependency fix
  * Fix localization evaluator tests
  * ci(pre-commit): autofix
  * Add missing includes and remove unnecessary quotes
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Wojciech Jaworski <wojciech.jaworski@robotec.ai>
* Contributors: Kenji Miyake, Mamoru Sobue, Satoshi OTA, Vincent Richard, djargot
