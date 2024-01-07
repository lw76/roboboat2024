import setuptools

setuptools.setup(
    name="asv",
    version="23.0.0",
    author="Team Inspiration",
    description="ASV Software",
    platforms="any",
    packages=["asv"],
    install_requires=[
        "numpy",
        "pyserial",
        "bluerobotics-ping",
        "python-dotenv",
        "pyyaml",
        "simple-pid",
        "circle-fit",
        "smbus",
        "crcmod",
        "shapely",
    ],
    # >= 2.7 (legacy, will drop support) or >= 3.6
    python_requires=">=2.7,!=3.0,!=3.1,!=3.2,!=3.3,!=3.4,!=3.5",
    extras_require={
        "dev": [
            "black",
            "pytest",
            "pytest-cov",
            "pytest-mock",
        ],
        "cv": [
            "opencv-python",
        ],
    },
)
