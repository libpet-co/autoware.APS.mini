^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_euclidean_cluster
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.45.0 (2025-05-22)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/notbot/bump_version_base
* chore: perception code owner update (`#10645 <https://github.com/autowarefoundation/autoware_universe/issues/10645>`_)
  * chore: update maintainers in multiple perception packages
  * Revert "chore: update maintainers in multiple perception packages"
  This reverts commit f2838c33d6cd82bd032039e2a12b9cb8ba6eb584.
  * chore: update maintainers in multiple perception packages
  * chore: add Kok Seang Tan as maintainer in multiple perception packages
  ---------
* Contributors: Taekjin LEE, TaikiYamada4

0.44.2 (2025-06-10)
-------------------

0.44.1 (2025-05-01)
-------------------

0.44.0 (2025-04-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(euclidean_cluster): add diagnostics warning when cluster skipped (`#10278 <https://github.com/autowarefoundation/autoware_universe/issues/10278>`_)
  * feat(euclidean_cluster): add diagnostics warning when cluster skipped due to excessive points from large objects
  * remove temporary code
  * style(pre-commit): autofix
  * feat(euclidean_cluster): diagnostics modified to remove redundant info
  * style(pre-commit): autofix
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* Contributors: Ryohsuke Mitsudome, lei.gu

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
* feat(autoware_euclidean_cluster)!: tier4_debug_msgs changed to autoware_internal_debug_msgs in autoware_euclidean_cluster (`#9873 <https://github.com/autowarefoundation/autoware_universe/issues/9873>`_)
  feat: tier4_debug_msgs changed to autoware_internal_debug_msgs in files perception/autoware_euclidean_cluster
  Co-authored-by: Ryohsuke Mitsudome <43976834+mitsudome-r@users.noreply.github.com>
* Contributors: Fumiya Watanabe, Vishal Chauhan

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
* fix(cpplint): include what you use - perception (`#9569 <https://github.com/autowarefoundation/autoware_universe/issues/9569>`_)
* 0.39.0
* update changelog
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(autoware_euclidean_cluster): fix bugprone-misplaced-widening-cast (`#9227 <https://github.com/autowarefoundation/autoware_universe/issues/9227>`_)
  fix: bugprone-misplaced-widening-cast
* Contributors: Esteve Fernandez, Fumiya Watanabe, M. Fatih Cırıt, Ryohsuke Mitsudome, Yutaka Kondo, kobayu858

0.39.0 (2024-11-25)
-------------------
* Merge commit '6a1ddbd08bd' into release-0.39.0
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* fix: fix ticket links to point to https://github.com/autowarefoundation/autoware_universe (`#9304 <https://github.com/autowarefoundation/autoware_universe/issues/9304>`_)
* chore(package.xml): bump version to 0.38.0 (`#9266 <https://github.com/autowarefoundation/autoware_universe/issues/9266>`_) (`#9284 <https://github.com/autowarefoundation/autoware_universe/issues/9284>`_)
  * unify package.xml version to 0.37.0
  * remove system_monitor/CHANGELOG.rst
  * add changelog
  * 0.38.0
  ---------
* fix(autoware_euclidean_cluster): fix bugprone-misplaced-widening-cast (`#9227 <https://github.com/autowarefoundation/autoware_universe/issues/9227>`_)
  fix: bugprone-misplaced-widening-cast
* Contributors: Esteve Fernandez, Yutaka Kondo, kobayu858

0.38.0 (2024-11-08)
-------------------
* unify package.xml version to 0.37.0
* refactor(autoware_point_types): prefix namespace with autoware::point_types (`#9169 <https://github.com/autowarefoundation/autoware_universe/issues/9169>`_)
* refactor(autoware_pointcloud_preprocessor): rework crop box parameters (`#8466 <https://github.com/autowarefoundation/autoware_universe/issues/8466>`_)
  * feat: add parameter schema for crop box
  * chore: fix readme
  * chore: remove filter.param.yaml file
  * chore: add negative parameter for voxel grid based euclidean cluster
  * chore: fix schema description
  * chore: fix description of negative param
  ---------
* refactor(pointcloud_preprocessor): prefix package and namespace with autoware (`#7983 <https://github.com/autowarefoundation/autoware_universe/issues/7983>`_)
  * refactor(pointcloud_preprocessor)!: prefix package and namespace with autoware
  * style(pre-commit): autofix
  * style(pointcloud_preprocessor): suppress line length check for macros
  * fix(pointcloud_preprocessor): missing prefix
  * fix(pointcloud_preprocessor): missing prefix
  * fix(pointcloud_preprocessor): missing prefix
  * fix(pointcloud_preprocessor): missing prefix
  * fix(pointcloud_preprocessor): missing prefix
  * refactor(pointcloud_preprocessor): directory structure (soft)
  * refactor(pointcloud_preprocessor): directory structure (hard)
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Kenzo Lobos Tsunekawa <kenzo.lobos@tier4.jp>
* refactor(euclidean_cluster): add package name prefix of autoware\_ (`#8003 <https://github.com/autowarefoundation/autoware_universe/issues/8003>`_)
  * refactor(euclidean_cluster): add package name prefix of autoware\_
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Amadeusz Szymko, Esteve Fernandez, Yi-Hsiang Fang (Vivid), Yutaka Kondo, badai nguyen

0.26.0 (2024-04-03)
-------------------
