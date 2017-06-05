/*[[[cog

import cog

]]]*/
//[[[end]]]

module pi_holes()
{
  
  /*[[[cog

  cog.outl()
  cog.outl( 'Pil = {0};'.format( PI_L ) )
  cog.outl( 'Piw = {0};'.format( PI_W ) )
  cog.outl()
  cog.outl( 'Pihr = {0};'.format( PI_HR ) )
  cog.outl()

  ]]]*/
  //[[[end]]]

  h1 = [ -Pil/2 +  3.5,  Piw/2 - 3.5 ];
  h2 = [  Pil/2 - 23.5,  Piw/2 - 3.5 ];
  h3 = [  Pil/2 - 23.5, -Piw/2 + 3.5 ];
  h4 = [ -Pil/2 +  3.5, -Piw/2 + 3.5 ];

  translate( h1 )
    circle( Pihr );
  translate( h2 )
    circle( Pihr );
  translate( h3 )
    circle( Pihr );
  translate( h4 )
    circle( Pihr );

}

module servo_holes()
{
  
  /*[[[cog

  cog.outl()
  cog.outl( 'Pil = {0};'.format( PI_L ) )
  cog.outl( 'Piw = {0};'.format( PI_W ) )
  cog.outl()
  cog.outl( 'Bl  = {0};'.format( B_L ) )
  cog.outl( 'Blp = {0};'.format( B_LP ) )
  cog.outl( 'Bw  = {0};'.format( B_W ) )
  cog.outl( 'Bwp = {0};'.format( B_WP ) )
  cog.outl()
  cog.outl( 'Din = {0};'.format( D_IN ) )
  cog.outl( 'Shr = {0};'.format( S_HR ) )
  cog.outl()

  ]]]*/
  //[[[end]]]

  v1 = [ -Bl /2,  Bwp/2 ];
  v2 = [ -Blp/2,  Bw /2 ];
  v3 = [  Blp/2,  Bw /2 ];
  v4 = [  Bl /2,  Bwp/2 ];
  v5 = [  Bl /2, -Bwp/2 ];
  v6 = [  Blp/2, -Bw /2 ];
  v7 = [ -Blp/2, -Bw /2 ];
  v8 = [ -Bl /2, -Bwp/2 ];


  p_inp_12 = (v1 + v2)/2;

  v_12  = v2 - v1;

  norm_12 = sqrt( v_12 * v_12 );
  u_12 = v_12 / norm_12;
  n_12 = [ u_12[1], -u_12[0] ];

  p_in_12  = p_inp_12 + Din * n_12;

  s_h_1 = p_in_12 - 8 * u_12;
  s_h_2 = p_in_12 + 8 * u_12;


  p_inp_34 = (v3 + v4)/2;

  v_34  = v4 - v3;

  norm_34 = sqrt( v_34 * v_34 );
  u_34 = v_34 / norm_34;
  n_34 = [ u_34[1], -u_34[0] ];

  p_in_34  = p_inp_34 + Din * n_34;

  s_h_3 = p_in_34 - 8 * u_34;
  s_h_4 = p_in_34 + 8 * u_34;


  p_inp_56 = (v5 + v6)/2;

  v_56  = v6 - v5;

  norm_56 = sqrt( v_56 * v_56 );
  u_56 = v_56 / norm_56;
  n_56 = [ u_56[1], -u_56[0] ];

  p_in_56  = p_inp_56 + Din * n_56;

  s_h_5 = p_in_56 - 8 * u_56;
  s_h_6 = p_in_56 + 8 * u_56;


  p_inp_78 = (v7 + v8)/2;

  v_78  = v8 - v7;

  norm_78 = sqrt( v_78 * v_78 );
  u_78 = v_78 / norm_78;
  n_78 = [ u_78[1], -u_78[0] ];

  p_in_78  = p_inp_78 + Din * n_78;

  s_h_7 = p_in_78 - 8 * u_78;
  s_h_8 = p_in_78 + 8 * u_78;


  translate( s_h_1 )
    circle( r = Shr );
  translate( s_h_2 )
    circle( r = Shr );
  translate( s_h_3 )
    circle( r = Shr );
  translate( s_h_4 )
    circle( r = Shr );
  translate( s_h_5 )
    circle( r = Shr );
  translate( s_h_6 )
    circle( r = Shr );
  translate( s_h_7 )
    circle( r = Shr );
  translate( s_h_8 )
    circle( r = Shr );

}

module body_shape()
{

  /*[[[cog

  cog.outl()
  cog.outl( 'Pil = {0};'.format( PI_L ) )
  cog.outl( 'Piw = {0};'.format( PI_W ) )
  cog.outl()
  cog.outl( 'Bl  = {0};'.format( B_L ) )
  cog.outl( 'Blp = {0};'.format( B_LP ) )
  cog.outl( 'Bw  = {0};'.format( B_W ) )
  cog.outl( 'Bwp = {0};'.format( B_WP ) )
  cog.outl()
  cog.outl( 'Pihr = {0};'.format( PI_HR ) )
  cog.outl( 'Shr  = {0};'.format( S_HR ) )
  cog.outl()

  ]]]*/
  //[[[end]]]

  v1 = [ -Bl /2,  Bwp/2 ];
  v2 = [ -Blp/2,  Bw /2 ];
  v3 = [  Blp/2,  Bw /2 ];
  v4 = [  Bl /2,  Bwp/2 ];
  v5 = [  Bl /2, -Bwp/2 ];
  v6 = [  Blp/2, -Bw /2 ];
  v7 = [ -Blp/2, -Bw /2 ];
  v8 = [ -Bl /2, -Bwp/2 ];

  difference()
  {
    polygon([v1,v2,v3,v4,v5,v6,v7,v8]);
    union()
    {
      pi_holes();
      servo_holes();
    }
  }

}

module body()
{

  /*[[[cog

  cog.outl()
  cog.outl( 'h = {0};'.format( H ) )
  cog.outl()  
  
  ]]]*/
  //[[[end]]]
    
  linear_extrude( height = h )
    body_shape();    

}

body();
