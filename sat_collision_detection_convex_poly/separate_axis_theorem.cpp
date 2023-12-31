#include<iostream>
#include<graphics.h>
#include<time.h>
#include<conio.h>
#include<math.h>
#include"concave_to_tris_ear_cut.h"
struct VEC2
{
    float X_Pos;
    float Y_Pos;
};
struct VEC3
{
    int r=255;
    int g=255;
    int b=255;
};
void draw_line(struct VEC2 P,struct VEC2 P1,struct VEC3 col);
char handleEvent();
double** Matrix_Multiplication(double**matA,double**matB,int rowA,int collumnA,int rowB,int collumnB);
void Transform_matrix_op(struct VEC2 pos, float angle,float scale,float t_x,float t_y,struct VEC2 m_pos,float *verts);

class POLY_COLLIDER
{
public:
    bool dynamic=false;
    float *verts;
    int verts_size;
    int m_id;
    bool in_collision;
    float angle=90;
    struct VEC2 m_pos={50,50};
    struct VEC2 forward_vec={0,1};
    POLY_COLLIDER() {};



    POLY_COLLIDER(int v_size,float *vert_data,class POLY_COLLIDER* m_colliders[],int unique_index,struct VEC2 w_position)
    {
        this->m_pos=w_position;
        this->m_id=unique_index;
        this->verts_size=v_size;
        this->verts=new float[verts_size];
        for(int i=0; i<verts_size; i++)
            this->verts[i]=vert_data[i];
        m_colliders[m_id]=this;
    }



    void update( class POLY_COLLIDER* colliders[],int num_of_colliders,float deltatime)
    {


        char inp;
        inp=handleEvent();
        //angle update
        float del_angle=0;
        if(inp=='A' || inp=='D')
        {
        del_angle=this->angle;
        this->angle+=(inp=='D')?100*deltatime:(inp=='A')?-100*deltatime:0;
        //angle=(int)angle%360;
        del_angle=angle-del_angle;
        this->forward_vec.X_Pos=cos((3.1415*angle)/180);
        this->forward_vec.Y_Pos=sin((3.1415*angle)/180);
        //this->forward_vec={cos(angle),sin(angle)};
        }
        float x_mov=(inp=='W')?deltatime*200*this->forward_vec.X_Pos:(inp=='S')?deltatime*-200*this->forward_vec.X_Pos:0;
        float y_mov=(inp=='W')?deltatime*200*this->forward_vec.Y_Pos:(inp=='S')?deltatime*-200*this->forward_vec.Y_Pos:0;


        if(this->m_id==0)
        {
        this->m_pos.X_Pos+=x_mov;
        this->m_pos.Y_Pos+=y_mov;
        for(int i=0; i<this->verts_size; i+=2)
        {
            float rad_angle=3.14159*del_angle/180;
            Transform_matrix_op({this->verts[i],this->verts[i+1]},(inp=='A' || inp=='D')?rad_angle:0,(inp=='F')?0.1:0,x_mov,y_mov,this->m_pos,&this->verts[i]);

        }
        }
        bool col;
        if(this->m_id!=0)
        col=this->check_collison_sat(colliders[0],this);

        for(int pol=0;pol<num_of_colliders;pol++)
        {
            if(colliders[pol]->m_id!=0 && colliders[pol]->m_id!=this->m_id)
            col|=this->check_collison_sat(colliders[pol],this);
        }

        if(col)
        this->render({255,255,0});
        else
        this->render({255,255,255});
    };





    bool check_collison_sat(class POLY_COLLIDER* colliders1,class POLY_COLLIDER* colliders2)
    {
        //for each poly
        class POLY_COLLIDER* shape1=colliders1;
        class POLY_COLLIDER* shape2=colliders2;
        float overlap=100000000;
        for(int i=0; i<2; i++)
        {

            if(i==1)
            {
                shape1=colliders2;
                shape2=colliders1;
            }

            for(int a=0; a<shape1->verts_size; a+=2)
            {
                int b=(a+2)%shape1->verts_size;
                float delY=shape1->verts[a+1]-shape1->verts[b+1];
                float delX=shape1->verts[a]-shape1->verts[b];
                struct VEC2 mid_point_of_axis= {(shape1->verts[a]+shape1->verts[b])/2.0, (shape1->verts[a]-shape1->verts[b])/2.0};
                struct VEC2 axis_n_to_e= {-delY,delX};
                axis_n_to_e.X_Pos/=sqrt(delY*delY+delX*delX);
                axis_n_to_e.Y_Pos/=sqrt(delY*delY+delX*delX);
                //for points in shape1
                float max_ov1=-10000000;
                float min_ov1=10000000;
                for(int j=0; j<shape1->verts_size; j+=2)
                {

                    //dot product btn norm vector and every point of shape 1
                    float q=((shape1->verts[j]-mid_point_of_axis.X_Pos)*axis_n_to_e.X_Pos)+((shape1->verts[j+1]-mid_point_of_axis.Y_Pos)*axis_n_to_e.Y_Pos);
                    //float q=(shape1->verts[j]*axis_n_to_e.X_Pos)+(shape1->verts[j+1]*axis_n_to_e.Y_Pos);
                    min_ov1=std::min(min_ov1,q);
                    max_ov1=std::max(max_ov1,q);
                }

                //for points in shape1
                float max_ov2=-10000000;
                float min_ov2=10000000;
                for(int j=0; j<shape2->verts_size; j+=2)
                {
                    //dot product btn norm vector and every point of shape 1

                    float q=((shape2->verts[j]-mid_point_of_axis.X_Pos)*axis_n_to_e.X_Pos)+((shape2->verts[j+1]-mid_point_of_axis.Y_Pos)*axis_n_to_e.Y_Pos);
                    //float q=(shape2->verts[j]*axis_n_to_e.X_Pos)+(shape2->verts[j+1]*axis_n_to_e.Y_Pos);
                    min_ov2=std::min(min_ov2,q);
                    max_ov2=std::max(max_ov2,q);
                }

                overlap=std::min(std::min(max_ov1,max_ov2)-std::max(min_ov1,min_ov2),overlap);

                if(!(max_ov2>=min_ov1 && max_ov1>=min_ov2))
                {
                    return false;
                }

            }

        }
        class POLY_COLLIDER* COLD=(shape1->m_id==0)?shape2:shape1;
        struct VEC2 d={shape2->m_pos.X_Pos-shape1->m_pos.X_Pos, shape2->m_pos.Y_Pos-shape1->m_pos.Y_Pos};
        float s=sqrt(d.X_Pos*d.X_Pos+d.Y_Pos*d.Y_Pos);
        for(int v=0;v<COLD->verts_size;v+=2)
        {
        COLD->verts[v]-=(overlap*d.X_Pos)/s;
        COLD->verts[v+1]-=(overlap*d.Y_Pos)/s;
        }
        COLD->m_pos.X_Pos-=(overlap*d.X_Pos)/s;
        COLD->m_pos.Y_Pos-=(overlap*d.Y_Pos)/s;


        return true;
    };



    void render(struct VEC3 colors)
    {
    for(int i=0;i<this->verts_size;i+=2)
    {
        int j=(i+2)%this->verts_size;
        draw_line({this->verts[i],this->verts[i+1]},{this->verts[j],this->verts[j+1]},colors);
    }

    draw_line({this->verts[4],this->verts[5]},this->m_pos,colors);
    };
};



int main()
{
    int gd = DETECT, gm;
    //initgraph(&gd, &gm, "C:\\TC\\BGI");
    initwindow(700, 700);
    srand(time(0));

    int num_of_colliders=0;

    class POLY_COLLIDER* colliders[100];
    float triangle_verticies[6]= {400,1,
                                  500,1,
                                  450,100
                                 };

    class POLY_COLLIDER* triangle=new POLY_COLLIDER(6,&triangle_verticies[0],colliders,0,{450,50});
    num_of_colliders++;
    float rect_verticies[8]= {400,400,
                               500,400,
                               500,500,
                               400,500
                             };

    class POLY_COLLIDER* rect=new POLY_COLLIDER(8,&rect_verticies[0],colliders,1,{450,450});
    num_of_colliders++;
    float pent_verticies[10]= {210,155,
                               200,202,
                               150,200,
                               140,156,
                               173,131
                             };

    class POLY_COLLIDER* pent=new POLY_COLLIDER(10,&pent_verticies[0],colliders,2,{176,170});
    num_of_colliders++;



/*
    //CONCAVE
    float concave_moon[20]={198,480,
                            189,465,
                            189,450,
                            200,439,
                            215,439,
                            231,448,
                            217,431,
                            196,429,
                            179,446,
                            181,467};


    int concave_arr_length=20;
    float **tris;
    int number_of_tris=(concave_arr_length/2)-2;

    tris=new float*[number_of_tris];
    for(int i=0;i<number_of_tris;i++)
    {
        tris[i]=new float[8]; //3 verts consisting x and y and the last index contains the mid point
    }
    concave_to_tris::split_concave_poly_into_tris(&concave_moon[0],&concave_arr_length,tris);

    //make this tris poly_colliders
    for(int i=0;i<number_of_tris;i++)
    {
    class POLY_COLLIDER* t1=new POLY_COLLIDER(6,tris[i],colliders,3+i,{tris[i][6],tris[i][7]});
    num_of_colliders++;
    }

    //end of CONCAVE DRAWING

*/

    colliders[0]->dynamic=true;

    float oldTime=clock();
    float deltaTime;
    //time_loop
    while(true)
    {

        swapbuffers();
        cleardevice();
        deltaTime=(clock()-oldTime)/1000.0;
        if(deltaTime>0.08)
            deltaTime=0.08;
        oldTime=clock();
        for(int i=0; i<num_of_colliders; i++)
        {
         colliders[i]->update(colliders,num_of_colliders,deltaTime);
        }


    }

    getch();
    closegraph();
    return 0;
}






void draw_line(struct VEC2 P,struct VEC2 P1,struct VEC3 col)
{
    double DX=abs(P1.X_Pos-P.X_Pos);
    double DY=abs(P1.Y_Pos-P.Y_Pos);
    if(DX>DY)//horizontal
    {
        if(P.X_Pos>P1.X_Pos)//checking if P is smaller than p1 always goes from low to high
        {
            draw_line(P1,P,col);
        }
        else
        {
            double m=(double)(P1.Y_Pos-P.Y_Pos)/(double)(P1.X_Pos-P.X_Pos);// calculating slope
            double yn=P.Y_Pos;
            double h=0.51;
            for(int i=P.X_Pos; i<=P1.X_Pos; i++)
            {
                yn+=m;
                putpixel(i,yn,COLOR(col.r,col.g,col.b));
            }
        }
    }
    else//vertical
    {
        if(P.Y_Pos>P1.Y_Pos)//checking if P is smaller than p1 always goes from low to high
        {
            draw_line(P1,P,col);
        }
        else
        {
            double m=(double)(P1.X_Pos-P.X_Pos)/(double)(P1.Y_Pos-P.Y_Pos);//calculating slope
            double xn=P.X_Pos;
            for(int i=P.Y_Pos; i<=P1.Y_Pos; i++)
            {
                xn+=m;
                putpixel(xn,i,COLOR(col.r,col.g,col.b));

            }
        }
    }
}







char handleEvent()
{
    // Check if a key is pressed

        int inputKey = _getch(); // Read the pressed key
        // Convert the char to integer
         int keyCode = static_cast<int>(inputKey);
        // Handle different key codes

        if(keyCode==87)
            return 'W';
        else if(keyCode==65)
            return 'A';
        else if(keyCode==83)
            return 'S';
        else if(keyCode==68)
            return 'D';
        else if(keyCode==70)
            return 'F';
        else
            return '1';
}



double** Matrix_Multiplication(double**matA,double**matB,int rowA,int collumnA,int rowB,int collumnB)
{
    double **MAT;
    MAT=new double*[rowA];
    for(int i=0; i<rowA; i++)
        MAT[i]=new double[collumnB];
    for(int i=0; i<rowA; i++)
        for(int j=0; j<collumnB; j++)
            MAT[i][j]=0;

    if(collumnA!=rowB)
    {
        return MAT;
    }

    for(int i=0; i<rowA; i++)
    {
        for(int j=0; j<collumnB; j++)
        {
            for(int k=0; k<collumnA; k++)
                MAT[i][j]+=matA[i][k]*matB[k][j];
        }
    }
    return MAT;
}


void Transform_matrix_op(struct VEC2 pos, float angle,float scale,float t_x,float t_y,struct VEC2 m_pos, float *verts)
{
    double **pos_mat;
    pos_mat=new double*[3];
    for(int i=0; i<3; i++)
    {
        pos_mat[i]=new double;
    }
    pos_mat[0][0]=pos.X_Pos;
    pos_mat[1][0]=pos.Y_Pos;
    pos_mat[2][0]=1;

    double **scale_mat;
    double **rot_mat_z;
    double **translate_mat;
    scale_mat=new double*[3];
    rot_mat_z=new double*[3];
    translate_mat=new double*[3];


    for(int i=0; i<3; i++)
    {
        scale_mat[i]=new double[3];
        rot_mat_z[i]=new double[3];
        translate_mat[i]=new double[3];
    }

    for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
        {
            scale_mat[i][j]=0;
            rot_mat_z[i][j]=0;
            translate_mat[i][j]=0;
        }


    scale_mat[0][0]=1+scale;
    scale_mat[1][1]=1+scale;
    scale_mat[2][2]=1;

    rot_mat_z[0][0]=cos(angle);
    rot_mat_z[0][1]=-sin(angle);
    rot_mat_z[1][0]=sin(angle);
    rot_mat_z[1][1]=cos(angle);
    rot_mat_z[2][2]=1;


    translate_mat[0][2]=-m_pos.X_Pos;
    translate_mat[1][2]=-m_pos.Y_Pos;
    translate_mat[2][2]=1;
    translate_mat[1][1]=1;
    translate_mat[0][0]=1;



    double **result_mat;
    result_mat=new double*[3];
    for(int i=0; i<3; i++)
        result_mat[i]=new double[3];


    double **row_vector;
    row_vector=new double*[3];
    for(int i=0; i<3; i++)
        row_vector[i]=new double;

    //scale
    row_vector=Matrix_Multiplication(translate_mat,pos_mat,3,3,3,1);
    row_vector=Matrix_Multiplication(scale_mat,row_vector,3,3,3,1);
    translate_mat[0][2]=m_pos.X_Pos;
    translate_mat[1][2]=m_pos.Y_Pos;
    row_vector=Matrix_Multiplication(translate_mat,row_vector,3,3,3,1);
    //rotation op
    translate_mat[0][2]=-m_pos.X_Pos;
    translate_mat[1][2]=-m_pos.Y_Pos;
    //points were translatrd to origin(v-m_pos)
    row_vector=Matrix_Multiplication(translate_mat,row_vector,3,3,3,1);//pos
    //rotation done against origin
    row_vector=Matrix_Multiplication(rot_mat_z,row_vector,3,3,3,1);
    translate_mat[0][2]=m_pos.X_Pos;
    translate_mat[1][2]=m_pos.Y_Pos;
    //translate back poits to world space(v' + m_pos)
    row_vector=Matrix_Multiplication(translate_mat,row_vector,3,3,3,1);
    //translate op
    //translate the points according ro required amount tx,ty
    translate_mat[0][2]=t_x;
    translate_mat[1][2]=t_y;
    row_vector=Matrix_Multiplication(translate_mat,row_vector,3,3,3,1);


    verts[0]=(float)row_vector[0][0];
    verts[1]=(float)row_vector[1][0];



}

