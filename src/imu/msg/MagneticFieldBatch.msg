  # Measurement of a Magnetic Field vector at a specific location.

 Header header                        		# timestamp is the time the
                                      		# field was measured
                                      		# frame_id is the location and orientation
                                      		# of the field measurement

 time[] stamps

 geometry_msgs/Vector3[] magnetic_fields 	# x, y, and z components of the
                                      		# field vector in Tesla
                                      		# If your sensor does not output 3 axes,
                                      		# put NaNs in the components not reported.