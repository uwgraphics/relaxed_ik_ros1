import test_utils

paths=["./rmoo_files/sawyer/sawyer_table1_relaxed_ik_ECA.poserr", \
    "./rmoo_files/sawyer/sawyer_table2_relaxed_ik_ECA.poserr", \
    "./rmoo_files/sawyer/sawyer_table1_moveit.poserr", \
    "./rmoo_files/sawyer/sawyer_table2_moveit.poserr", \
    "./rmoo_files/sawyer/sawyer_table1_relaxed_ik_ECA.roterr", \
    "./rmoo_files/sawyer/sawyer_table2_relaxed_ik_ECA.roterr", \
    "./rmoo_files/sawyer/sawyer_table1_moveit.roterr", \
    "./rmoo_files/sawyer/sawyer_table2_moveit.roterr"]

test_utils.plot_error(paths)