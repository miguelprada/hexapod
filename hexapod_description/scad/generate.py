#!/usr/bin/env python

import subprocess
import os

import yaml

import argparse

import numpy as np
from math import pi


DEFAULT_OUTFILE = os.path.expanduser( '~/.ros/hexapod_description' )


class Logger:

    def __init__( self, print_debug=False, print_ddebug=False, info_col='\033[92m', debug_col='\033[93m', ddebug_col='\033[94m' ):

        self._info_col  = info_col
        self._debug_col = debug_col
        self._ddebug_col = ddebug_col

        self._reset_col = '\033[0m'

        self._print_debug = print_debug
        self._print_ddebug = print_debug and print_ddebug


    def info( self, text ):
        self._print_with_col( self._info_col, text )

    def debug( self, text ):
        if self._print_debug:
            self._print_with_col( self._debug_col, text )

    def ddebug( self, text ):
        if self._print_ddebug:
            self._print_with_col( self._ddebug_col, text )

    def _print_with_col( self, col, text ):
        print( '{0}{1}{2}'.format( col, text, self._reset_col ) )



def make_temp_dir( logger, prefix ):

    try:
        os.makedirs( prefix )
        logger.info( 'Creating directory {0}'.format( prefix ) )
    except OSError as e:
        logger.info( 'Not creating any directory, {0} already exists'.format( prefix ) )



def generate_scad( logger, data, name, temp_prefix ):

    scad_filepath = '{0}/body.scad'.format( temp_prefix )

    logger.info( 'Generating: {0}'.format( scad_filepath ) )

    import rospkg
    rospack = rospkg.RosPack()

    scad_generation_command = ['cog.py',
                               '-D', 'PI_L={0}'.format(data[name]['pi_l']),
                               '-D', 'PI_W={0}'.format(data[name]['pi_w']),
                               '-D', 'B_L={0}'.format(data[name]['b_l']),
                               '-D', 'B_LP={0}'.format(data[name]['b_lp']),
                               '-D', 'B_W={0}'.format(data[name]['b_w']),
                               '-D', 'B_WP={0}'.format(data[name]['b_wp']),
                               '-D', 'PI_HR={0}'.format(data[name]['pi_hr']),
                               '-D', 'D_IN={0}'.format(data[name]['d_in']),
                               '-D', 'S_HR={0}'.format(data[name]['s_hr']),
                               '-D', 'H={0}'.format(data[name]['h']),
                               '{0}/scad/body.cog.scad'.format( rospack.get_path( 'hexapod_description' ) ) ]

    logger.debug( 'Running: {0} > {1}'.format( ' '.join( scad_generation_command ), scad_filepath ) )

    scad_file = open( scad_filepath, 'w' )
    subprocess.call( scad_generation_command, stdout=scad_file )

    return scad_filepath



def generate_ascii_stl( logger, name, temp_prefix, scad_filepath ):

    ascii_stl_filepath = '{0}/body.stl'.format( temp_prefix, name )

    logger.info( 'Generating: {0}'.format( ascii_stl_filepath ) )

    ascii_stl_generation_command = [ 'openscad',
                                     '-o', ascii_stl_filepath,
                                     scad_filepath]

    logger.debug( 'Running: {0}'.format( ' '.join( ascii_stl_generation_command ) ) )

    subprocess.call( ascii_stl_generation_command )

    return ascii_stl_filepath



def convert_ascii_to_binary_stl( logger, ascii_stl_filepath ):

    ascii_stl_filename = ascii_stl_filepath.split('/')[-1]

    binary_stl_filename = '-binary.'.join( ascii_stl_filename.split('.') )

    binary_stl_filepath = '/'.join( ascii_stl_filepath.split('/')[:-1] + [binary_stl_filename] )

    logger.info( 'Generating: {0}'.format( binary_stl_filepath ) )

    import rospkg
    rospack = rospkg.RosPack()

    binary_stl_generation_command = [ 'ruby', '{0}/scad/convertSTL.rb'.format( rospack.get_path( 'hexapod_description' ) ), ascii_stl_filepath ]

    logger.debug( 'Running: {0}'.format( binary_stl_generation_command ) )

    subprocess.call( binary_stl_generation_command )

    return binary_stl_filepath



def compute_leg_transform( logger, v1, v2, d_in, h ):

    logger.ddebug( 'Vertex 1: {0}'.format( v1 ) )
    logger.ddebug( 'Vertex 2: {0}'.format( v2 ) )

    p_inp_12 = (v1 + v2)/2.0

    logger.ddebug( 'P in\'   : {0}'.format( p_inp_12 ) )

    v_12  = v2 - v1

    logger.ddebug( 'V12     : {0}'.format( v_12 ) )

    norm_12 = np.linalg.norm( v_12 )

    u_12 = v_12 / norm_12;
    n_12 = np.dot( np.array([[0,1],[-1,0]]), u_12 )

    logger.ddebug( 'U12     : {0}'.format( u_12 ) )
    logger.ddebug( 'N12     : {0}'.format( n_12 ) )

    p_in_12  = p_inp_12 + d_in * n_12

    pos = np.append( p_in_12, h + 16 )
    pos = pos / 1000.0

    rz = np.arctan2( u_12[1], u_12[0] )

    logger.debug( 'Leg angle: {0}'.format( rz ) )

    rot = np.array( [0,0,rz+pi/2] )

    return pos, rot



def generate_urdf( logger, data, name, temp_prefix, binary_stl_filepath ):

    xacro_filepath = '{0}/tetrapod.urdf.xacro'.format( temp_prefix, name )

    logger.info( 'Generating: {0}'.format( xacro_filepath ) )

    vertices = []

    vertices.append( np.array( [ -data[name][ 'b_l']/2.0,  data[name]['b_wp']/2.0 ] ) )
    vertices.append( np.array( [ -data[name]['b_lp']/2.0,  data[name][ 'b_w']/2.0 ] ) )
    vertices.append( np.array( [  data[name]['b_lp']/2.0,  data[name][ 'b_w']/2.0 ] ) )
    vertices.append( np.array( [  data[name][ 'b_l']/2.0,  data[name]['b_wp']/2.0 ] ) )
    vertices.append( np.array( [  data[name][ 'b_l']/2.0, -data[name]['b_wp']/2.0 ] ) )
    vertices.append( np.array( [  data[name]['b_lp']/2.0, -data[name][ 'b_w']/2.0 ] ) )
    vertices.append( np.array( [ -data[name]['b_lp']/2.0, -data[name][ 'b_w']/2.0 ] ) )
    vertices.append( np.array( [ -data[name][ 'b_l']/2.0, -data[name]['b_wp']/2.0 ] ) )

    leg_fr_pos, leg_fr_rot = compute_leg_transform( logger, vertices[0], vertices[1], data[name]['d_in'], data[name]['h'] )
    leg_rr_pos, leg_rr_rot = compute_leg_transform( logger, vertices[2], vertices[3], data[name]['d_in'], data[name]['h'] )
    leg_rl_pos, leg_rl_rot = compute_leg_transform( logger, vertices[4], vertices[5], data[name]['d_in'], data[name]['h'] )
    leg_fl_pos, leg_fl_rot = compute_leg_transform( logger, vertices[6], vertices[7], data[name]['d_in'], data[name]['h'] )

    import rospkg
    rospack = rospkg.RosPack()

    xacro_generation_command = ['cog.py',
                                '-D', 'FILENAME={0}'.format( binary_stl_filepath ),
                                '-D', 'LEG_FR_POS="{0} {1} {2}"'.format( leg_fr_pos[0], leg_fr_pos[1], leg_fr_pos[2] ),
                                '-D', 'LEG_FR_ROT="{0} {1} {2}"'.format( leg_fr_rot[0], leg_fr_rot[1], leg_fr_rot[2] ),
                                '-D', 'LEG_RR_POS="{0} {1} {2}"'.format( leg_rr_pos[0], leg_rr_pos[1], leg_rr_pos[2] ),
                                '-D', 'LEG_RR_ROT="{0} {1} {2}"'.format( leg_rr_rot[0], leg_rr_rot[1], leg_rr_rot[2] ),
                                '-D', 'LEG_RL_POS="{0} {1} {2}"'.format( leg_rl_pos[0], leg_rl_pos[1], leg_rl_pos[2] ),
                                '-D', 'LEG_RL_ROT="{0} {1} {2}"'.format( leg_rl_rot[0], leg_rl_rot[1], leg_rl_rot[2] ),
                                '-D', 'LEG_FL_POS="{0} {1} {2}"'.format( leg_fl_pos[0], leg_fl_pos[1], leg_fl_pos[2] ),
                                '-D', 'LEG_FL_ROT="{0} {1} {2}"'.format( leg_fl_rot[0], leg_fl_rot[1], leg_fl_rot[2] ),
                                '-D', 'H={0}'.format( data[name]['h'] ),
                                '{0}/scad/tetrapod.cog.urdf.xacro'.format( rospack.get_path( 'hexapod_description' ) ) ]

    logger.debug( 'Running: {0} > {1}'.format( ' '.join( xacro_generation_command ), xacro_filepath ) )

    xacro_file = open( xacro_filepath, 'w' )
    subprocess.call( xacro_generation_command, stdout=xacro_file )


    urdf_filepath = '{0}/tetrapod.urdf'.format( temp_prefix, name )

    logger.info( 'Generating: {0}'.format( urdf_filepath ) )

    urdf_generation_command = ['rosrun', 'xacro', 'xacro', rospack.get_path( 'hexapod_description' )+'/model/gen_tetrapod.urdf.xacro', 'model_path:={0}'.format( '/'.join( xacro_filepath.split('/')[:-1] ) ) ]

    logger.debug( 'Running: {0} > {1}'.format( ' '.join( urdf_generation_command ), urdf_filepath ) )

    urdf_file = open( urdf_filepath, 'w' )
    subprocess.call( urdf_generation_command, stdout=urdf_file )

    return urdf_filepath



def visualize_in_rviz( logger, urdf_filepath ):

    logger.info( 'Visualization of the generated model has been broken by recent changes.' )
    return

    display_command = [ 'roslaunch', 'urdf_tutorial', 'gen_xacrodisplay.launch', 'model:={0}'.format( urdf_filepath ) ]

    logger.debug( 'Running: {0}'.format( display_command ) )

    subprocess.call( display_command )



def process_dimension_set( logger, data, name, temp_prefix, visualize ):

    logger.info( '------------------------------------------------------' )
    logger.info( 'Processing dimension set: {0}'.format( name ) )

    scad_filepath = generate_scad( logger, data, name, temp_prefix )

    ascii_stl_filepath = generate_ascii_stl( logger, name, temp_prefix, scad_filepath )

    binary_stl_filepath = convert_ascii_to_binary_stl( logger, ascii_stl_filepath )

    urdf_filepath = generate_urdf( logger, data, name, temp_prefix, binary_stl_filepath )

    if visualize:

        visualize_in_rviz( logger, urdf_filepath )



def main( datafile, outdir, verbose, display, names ):

    logger = Logger( print_debug = args.verbose>=1, print_ddebug = args.verbose>=2 )

    data = yaml.load( open( datafile ) )

    for name in data:

        if names and (name not in names):
            continue

        namedir = outdir + '/' + name

        make_temp_dir( logger, namedir )

        process_dimension_set( logger, data, name, namedir, display )



if __name__ == '__main__':

    parser = argparse.ArgumentParser( description='Generate scad, stl and URDF files for hexapod body reading dimensions from a YAML file', formatter_class=argparse.ArgumentDefaultsHelpFormatter )

    parser.add_argument( 'data_file', help='YAML file with the dimensions data' )
    parser.add_argument( '-o', '--outdir', nargs='?', default=DEFAULT_OUTFILE, help='Folder to store the generated files' )
    parser.add_argument( '-v', '--verbose', action='count', help='Increase script verbosity' )
    parser.add_argument( '-d', '--display', action='store_true', help='Display result of generated files in rviz' )
    parser.add_argument( '-n', '--name', action='append', help='Name of the dimension sets to generate' )

    args = parser.parse_args()

    main( args.data_file, args.outdir, args.verbose, args.display, args.name )
