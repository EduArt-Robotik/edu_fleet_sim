from distutils.core import setup

setup(
    version='0.1.0',
    scripts=['scripts/fleet_sim_node.py'],
    packages=['edu_fleet_sim'],
    package_dir={'': 'src'},
    data_files=[
        ('share/edu_fleet_sim/images', [
            'images/mecanum_crash_2.png',
            'images/mecanum_crash.png',
            'images/mecanum_edu_1.png',
            'images/mecanum_edu_2.png',
            'images/mecanum_ohm_1.png',
            'images/mecanum_ohm_2.png',
            'images/mecanum.png',
            'images/screenshot.png',
        ])
    ]
)
