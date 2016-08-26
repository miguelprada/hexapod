#!/usr/bin/env python

import subprocess
import os

import yaml

import argparse


DEFAULT_OUTFILE = '/tmp/hxpd'


class Logger:

    def __init__( self, print_debug=False, info_col='\033[92m', debug_col='\033[93m' ):

        self._info_col  = info_col
        self._debug_col = debug_col

        self._reset_col = '\033[0m'

        self._print_debug = print_debug


    def info( self, text ):
        self._print_with_col( self._info_col, text )

    def debug( self, text ):
        if self._print_debug:
            self._print_with_col( self._debug_col, text )

    def _print_with_col( self, col, text ):
        print( '{0}{1}{2}'.format( col, text, self._reset_col ) )



logger = None



def make_temp_dir( prefix ):

    global logger

    try:
        os.mkdir( prefix )
        logger.info( 'Creating directory {0}'.format( prefix ) )
    except OSError as e:
        logger.info( 'Not creating any directory, {0} already exists'.format( prefix ) )



def generate_scad( data, name, temp_prefix ):

    scad_filename = '{0}/body_{1}.scad'.format( temp_prefix, name )

    logger.info( 'Generating: {0}'.format( scad_filename ) )

    scad_generation_command = ['cog.py',
                               '-D', 'PI_L={0}'.format(data[name]['pi_l']),
                               '-D', 'PI_W={0}'.format(data[name]['pi_w']),
                               '-D', 'B_L={0}'.format(data[name]['b_l']),
                               '-D', 'B_LP={0}'.format(data[name]['b_lp']),
                               '-D', 'B_W={0}'.format(data[name]['b_w']),
                               '-D', 'B_WP={0}'.format(data[name]['b_wp']),
                               '-D', 'PI_HR={0}'.format(data[name]['pi_hr']),
                               '-D', 'S_HR={0}'.format(data[name]['s_hr']),
                               '-D', 'H={0}'.format(data[name]['h']),
                               'body.cog.scad' ]

    logger.debug( 'Running: {0} > {1}'.format( ' '.join( scad_generation_command ), scad_filename ) )

    scad_file = open( scad_filename, 'w' )
    subprocess.call( scad_generation_command, stdout=scad_file )

    return scad_filename



def generate_ascii_stl( data, name, temp_prefix, scad_filename ):

    ascii_stl_filename = '{0}/body_{1}.stl'.format( temp_prefix, name )

    logger.info( 'Generating: {0}'.format( ascii_stl_filename ) )

    ascii_stl_generation_command = [ 'openscad',
                                     '-o', ascii_stl_filename,
                                     scad_filename]
                         
    logger.debug( 'Running: {0}'.format( ' '.join( ascii_stl_generation_command ) ) )

    subprocess.call( ascii_stl_generation_command )

    return ascii_stl_filename



def convert_ascii_to_binary_stl( data, name, temp_prefix, ascii_stl_filename ):

    binary_stl_filename = '-binary.'.join( ascii_stl_filename.split('.') )

    logger.info( 'Generating: {0}'.format( binary_stl_filename ) )

    binary_stl_generation_command = [ 'ruby', 'convertSTL.rb', ascii_stl_filename ]

    logger.debug( 'Running: {0}'.format( binary_stl_generation_command ) )

    subprocess.call( binary_stl_generation_command )

    return binary_stl_filename



def generate_urdf( data, name, temp_prefix, binary_stl_filename ):

    urdf_filename = '{0}/body_{1}.urdf'.format( temp_prefix, name )

    logger.info( 'Generating: {0}'.format( urdf_filename ) )

    urdf_generation_command = ['cog.py',
                               '-D', 'FILENAME={0}'.format(binary_stl_filename),
                               'body.cog.urdf' ]

    logger.debug( 'Running: {0} > {1}'.format( ' '.join( urdf_generation_command ), urdf_filename ) )

    urdf_file = open( urdf_filename, 'w' )
    subprocess.call( urdf_generation_command, stdout=urdf_file )

    return urdf_filename



def visualize_in_rviz( data, name, temp_prefix, urdf_filename ):

    display_command = [ 'roslaunch', 'urdf_tutorial', 'display.launch', 'model:={0}'.format( urdf_filename ) ]

    logger.debug( 'Running: {0}'.format( display_command ) )

    subprocess.call( display_command )



def process_dimension_set( data, name, temp_prefix, visualize ):

    logger.info( '------------------------------------------------------' )
    logger.info( 'Processing dimension set: {0}'.format( name ) )
    
    scad_filename = generate_scad( data, name, temp_prefix )  

    ascii_stl_filename = generate_ascii_stl( data, name, temp_prefix, scad_filename )

    binary_stl_filename = convert_ascii_to_binary_stl( data, name, temp_prefix, ascii_stl_filename )

    urdf_filename = generate_urdf( data, name, temp_prefix, binary_stl_filename )

    if visualize:

        visualize_in_rviz( data, name, temp_prefix, urdf_filename )



def main( datafile, outdir, display ):

    make_temp_dir( outdir )

    data = yaml.load( open( datafile ) )

    for name in data:

        process_dimension_set( data, name, outdir, display )
  


if __name__ == '__main__':

    parser = argparse.ArgumentParser( description='Generate scad, stl and URDF files for hexapod body reading dimensions from a YAML file', formatter_class=argparse.ArgumentDefaultsHelpFormatter )

    parser.add_argument( 'data_file', help='YAML file with the dimensions data' )
    parser.add_argument( '-o', '--outdir', nargs='?', default=DEFAULT_OUTFILE, help='Folder to store the generated files' )
    parser.add_argument( '-v', '--verbose', action='store_true', help='Increase script verbosity' )
    parser.add_argument( '-d', '--display', action='store_true', help='Display result of generated files in rviz' )

    args = parser.parse_args()

    global logger
    logger = Logger( print_debug = args.verbose )

    main( args.data_file, args.outdir, args.display )
