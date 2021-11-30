^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package raptor_dbw_can
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2021-11-30)
------------------
* Fix accel and steer ignore to not disable the system on overrides (`#11 <https://github.com/NewEagleRaptor/raptor-dbw-ros2/issues/11>`_)
  * Also add Driver Ignore control for articulation & dump bed
* Contributors: Benjamin Gervan, neweagleraptor

1.1.4 (2021-08-25)
------------------
* Updated DBC - moved EStop signals
* Contributors: neweagleraptor

1.1.3 (2021-08-24)
------------------
* Updated DBC for new messages & signals
* Contributors: neweagleraptor

1.1.2 (2021-07-13)
------------------
* Update to control dump trucks
* Contributors: neweagleraptor

1.1.1 (2021-07-13)
------------------

1.1.0 (2021-06-14)
------------------
* Misc. cleanup & comment improvement
* Update DBW node to use params from launch file
* Fix unit conversion issues - steering was in radians, should be degrees
* Contributors: neweagleraptor

1.0.0 (2021-03-22)
------------------
* Initial release
* Contributors: Joshua Whitley, New Eagle, dev, neweagleraptor
