^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_ez_publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2016-04-11)
------------------
* Fix save/load dialog
* Search from deeper topic name
* Contributors: Takashi Ogura

0.3.0 (2014-08-25)
------------------
* Add save/load button in config dialog
* Support yaml setting file
* Support action in devel environment
* update some log messages

0.2.0 (2014-07-19)
------------------
* Add tf for dependency
* Refactoring
  - Use base_widget
  - divided widgets into widget/
  - moved publishers into publisher/
  - rpy to quaternion_module/
* do not create header/seq
* Convert Quaternion to RPY

0.1.0 (2014-07-13)
------------------
* Change default for double 1.0
* Support tf and header
* fix bug with array
* fix type bug

0.0.5 (2014-07-11)
------------------
* Fix test wait

0.0.4 (2014-07-11)
------------------
* Update for indigo
* Fix NoneType error for make_topic_string
* Clean codes (applied pep8)
* Support byte
* Add sphinx doc

0.0.3 (2014-07-05)
------------------
* Add tests that uses ros
* Support array of non builtin type
* add build status in README.md
* Add config dialog, reload button

0.0.2 (2014-06-29)
------------------
* MoveUP/Down and use icon
* Fix cmake (remove python packages from find_package)
* Support repeat publish
* support Header
* add queue_size for publisher for indigo
* for python3

0.0.1 (2014-06-29)
------------------
* more easy topic publisher
