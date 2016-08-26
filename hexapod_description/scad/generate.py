#!/usr/bin/env python

import subprocess
import os

import yaml

DATA = yaml.load(open('dimensions.yaml'))

TEMP_PREFIX = '/tmp/hxpd'

DO_DEBUG = True

def INFO(text):
    print( '\033[92m{0}\033[0m'.format(text) )

def DEBUG(text):
    if DO_DEBUG:
        print( '\033[93m{0}\033[0m'.format(text) )

try:
    os.mkdir(TEMP_PREFIX)
    INFO( 'Creating directory {0}'.format(TEMP_PREFIX) )
except OSError as e:
    INFO( 'Not creating any directory, {0} already exists'.format(TEMP_PREFIX) )

for name in DATA:
  

    INFO( '------------------------------------------------------' )
    INFO( 'Processing dimension set: {0}'.format(name) )
    
  

    scad_filename = '{0}/body_{1}.scad'.format(TEMP_PREFIX,name)

    INFO( 'Generating: {0}'.format(scad_filename) )

    scad_generation_command = ['cog.py',
                               '-D', 'PI_L={0}'.format(DATA[name]['pi_l']),
                               '-D', 'PI_W={0}'.format(DATA[name]['pi_w']),
                               '-D', 'B_L={0}'.format(DATA[name]['b_l']),
                               '-D', 'B_W={0}'.format(DATA[name]['b_w']),
                               '-D', 'B_WP={0}'.format(DATA[name]['b_wp']),
                               '-D', 'H={0}'.format(DATA[name]['h']),
                               'body.cog.scad' ]

    DEBUG( 'Running: {0} > {1}'.format( ' '.join( scad_generation_command ), scad_filename ) )

    scad_file = open( scad_filename, 'w' )
    subprocess.call( scad_generation_command, stdout=scad_file )



    ascii_stl_filename = '{0}/body_{1}.stl'.format( TEMP_PREFIX, name )

    INFO( 'Generating: {0}'.format(ascii_stl_filename))

    ascii_stl_generation_command = [ 'openscad',
                                     '-o', ascii_stl_filename,
                                     scad_filename]
                         
    DEBUG( 'Running: {0}'.format( ' '.join( ascii_stl_generation_command ) ) )

    subprocess.call( ascii_stl_generation_command )



    binary_stl_filename = '-binary.'.join( ascii_stl_filename.split('.') )

    INFO( 'Generating: {0}'.format( binary_stl_filename ) )

    binary_stl_generation_command = [ 'ruby', 'convertSTL.rb', ascii_stl_filename ]

    DEBUG( 'Running: {0}'.format( binary_stl_generation_command ) )

    subprocess.call( binary_stl_generation_command )



    urdf_filename = '{0}/body_{1}.urdf'.format(TEMP_PREFIX,name)

    INFO( 'Generating: {0}'.format(urdf_filename) )

    urdf_generation_command = ['cog.py',
                               '-D', 'FILENAME={0}'.format(binary_stl_filename),
                               'body.cog.urdf' ]

    DEBUG( 'Running: {0} > {1}'.format( ' '.join( urdf_generation_command ), urdf_filename ) )

    urdf_file = open( urdf_filename, 'w' )
    subprocess.call( urdf_generation_command, stdout=urdf_file )



    display_command = [ 'roslaunch', 'urdf_tutorial', 'display.launch', 'model:={0}'.format( urdf_filename ) ]

    DEBUG( 'Running: {0}'.format( display_command ) )

    subprocess.call( display_command )
