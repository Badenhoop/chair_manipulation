#!/usr/bin/env python3
import os
import os.path as osp
from pathlib import Path
import warnings

import numpy as np
import trimesh
from jinja2 import Template
import requests
from zipfile import ZipFile



def render_jinja_template(jinja_file, dst_file, var_dict):
    template = Template(open(jinja_file).read())
    template.stream(**var_dict).dump(dst_file)


def find_files(topdir, extensions=[".mtl", ".obj"]):
    for dirpath, dirnames, filenames in os.walk(topdir):
        if all(any(f.endswith(ext) for f in filenames) for ext in extensions):
            objfile = [f for f in filenames if f.endswith(".obj")]
            yield osp.join(dirpath, objfile[0])


def trimesh_mass_properties(objfile, density):
    scene = trimesh.load(objfile)
    mesh = scene.convex_hull
    mesh.density = density
    return dict(inertia=mesh.mass_properties['inertia'],
                center=mesh.mass_properties['center_mass'],
                mass=mesh.mass_properties['mass'])


def parse_sketchup_file_units(objfile):
    for line in open(objfile):
        if line.startswith("# File units = "):
            _, unitstr = line.split("=")
            unitstr = unitstr.strip()
            return unitstr


def unitstr2rescale_factor(unitstr,
                           conversions=dict(meters=1,
                                            centimeters=0.01,
                                            millimeters=0.001,
                                            inches=0.0254)):
    return conversions[unitstr]


def mesh_mass_properties_from_trimesh(objfile, density=10):
    mass_props = trimesh_mass_properties(objfile, density)
    unit = unitstr2rescale_factor(parse_sketchup_file_units(objfile))
    return dict(scale=(np.ones(3) * unit).tolist(),
                mass=mass_props['mass'] * unit**3 ,
                center=(mass_props['center'] * unit).tolist(),
                inertia=(mass_props['inertia'] * unit**2).tolist())


def unzip_file(zipfile, dest_dir):
    ZipFile(zipfile).extractall(path=dest_dir)


def download_ikea_meshes(
        tofile,
        url="http://ikea.csail.mit.edu/zip/IKEA_models.zip"):
    r = requests.get(url)
    print("Downloading IKEA ", end="")
    with open(tofile, "wb") as fd:
        for chunk in r.iter_content(chunk_size=4098):
            print(".", end="")
            fd.write(chunk)
    print("Done")


def download_and_unzip_ikea_meshes(dst_dir):
    zip_file = dst_dir + ".zip"
    Path(zip_file).parent.mkdir(parents=True, exist_ok=True)
    download_ikea_meshes(zip_file)
    unzipped_dir = dst_dir + "_unzipped"
    unzip_file(zip_file, unzipped_dir)
    os.rename(osp.join(unzipped_dir, "IKEA"), dst_dir)
    os.remove(zip_file)
    os.removedirs(unzipped_dir)
    return True


def relpath(path,
            refdir=lambda : osp.dirname(__file__) or "."):
    return osp.join(refdir(), path)


def main(topdir="meshes",
         template_fmt="{file}.jinja",
         dest_dir=".",
         generated_files=["model.config", "model.sdf"]):
    if os.getcwd() not in os.getenv("GAZEBO_MODEL_PATH").split(":"):
        warnings.warn("""Please run from a directory that you have added or
        will add to GAZEBO_MODEL_PATH. The mesh file uri's generated are
        relative to model:// which depends on
        GAZEBO_MODEL_PATH""".replace("\n", " "))
    topdir = relpath(topdir)
    if not osp.exists(topdir):
        download_and_unzip_ikea_meshes(topdir)
    for objfile in find_files(topdir):
        objfileparts = objfile.split(osp.sep)
        ikea_names = [fp for fp in objfileparts if fp.startswith("IKEA_")]
        ikea_model_dir = osp.join(dest_dir, ikea_names[0])
        objprops = mesh_mass_properties_from_trimesh(objfile)
        Path(ikea_model_dir).mkdir(parents=True, exist_ok=True)
        for f in generated_files:
            file_to_gen = relpath(f)
            render_jinja_template(
                template_fmt.format(file=file_to_gen),
                osp.join(ikea_model_dir, f),
                dict(IKEA_NAMES=ikea_names,
                     OBJFILENAME=objfile,
                     SCALE=objprops["scale"],
                     CENTER=objprops["center"],
                     MASS=objprops["mass"],
                     INERTIA=objprops["inertia"]))


if __name__ == '__main__':
    main()
