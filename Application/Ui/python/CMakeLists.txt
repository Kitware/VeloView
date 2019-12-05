include(PythonModuleInstaller)

python_module_install(NAME lidarview
                      FILES
                      lidarview/__init__.py
                      lidarview/applogic.py
                      lidarview/gridAdjustmentDialog.py
                      lidarview/kiwiviewerExporter.py
                      lidarview/timer.py
                      lidarview/aboutDialog.py
                      lidarview/DTMFilter/__init__.py
                      lidarview/DTMFilter/DTMPythonFilter_Script.py
                      lidarview/DTMFilter/DTM.py
                      lidarview/visualizationTools.py
                      )
