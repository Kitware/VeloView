# Copy the scripts when installing

set(install_destination ${VV_INSTALL_LIBRARY_DIR}/site-packages)

set(scripts 
        example_temporal_animation.py
        example_non_temporal_animation.py
        temporal_animation_cue.py
        lib/camera_path.py
    )

install(FILES ${scripts} DESTINATION ${install_destination})
