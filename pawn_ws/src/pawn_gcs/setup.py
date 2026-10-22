from setuptools import setup, find_packages

package_name = "pawn_gcs"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "pawn_gcs = pawn_gcs.app:main",
        ],
    },
)