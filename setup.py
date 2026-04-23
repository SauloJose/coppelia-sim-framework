"""Setup file for CoppeliaSim Framework package."""

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="brainbyte",
    version="1.1.12",
    author="Saulo José",
    author_email="saulo-jose12@hotmail.com",
    description="Professional Python framework for CoppeliaSim robot simulations",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/SauloJose/coppelia-sim-framework",
    packages=find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.8",
    install_requires=[
        "coppeliasim-zmqremoteapi-client",
        "numpy",
        "matplotlib",
        "keyboard",
    ],
    extras_require={
        "dev": [
            "pytest>=7.0",
            "pytest-cov",
            "black",
            "flake8",
            "mypy",
            "sphinx",
            "sphinx-rtd-theme",
        ],
    },
)
