test:
	make -f ./make_dir/makefile_run
	make -f ./make_dir/makefile_radar_no
	make -f ./make_dir/makefile_cal_radar

clean:
	make clean -f ./make_dir/makefile_run
	make clean -f ./make_dir/makefile_radar_no
	make clean -f ./make_dir/makefile_cal_radar
