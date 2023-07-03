#include<iostream>
#include<math.h>
#include<graphics.h>
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

bool is_clock_wise(struct VEC2 A,struct VEC2 B,struct VEC2 C);
bool is_point_inside_triangle(struct VEC2 A,struct VEC2 B,struct VEC2 C,struct VEC2 P);
void adjust_array(float *a,int* array_length,int drop_index);
void form_tris(float *tris);
void draw_line(struct VEC2 P,struct VEC2 P1,struct VEC3 col);
double** Matrix_Multiplication(double**matA,double**matB,int rowA,int collumnA,int rowB,int collumnB);
void Transform_matrix_op(struct VEC2 pos, float angle,float scale,float t_x,float t_y,struct VEC2 m_pos, float *verts);
void split_concave_poly_into_tris(float *a,int *ar_length,float *tris[]);


int main()
{
    int gd = DETECT, gm;
    //initgraph(&gd, &gm, "C:\\TC\\BGI");
    initwindow(700, 700);
    //concave poly
    float a[10]={250,465,
                337,397,
                263,272,
                163,298,
                120,420};
    /*{198,480,
                 189,465,
                 189,450,
                 200,439,
                 215,439,
                 231,448,
                 217,431,
                 196,429,
                 179,446,
                 181,467
                 };*/

for(int i=0;i<10;i+=2)
Transform_matrix_op({a[i],a[i+1]},-45,0,0,-100,{200,450},&a[i]);

for(int i=0;i<10;i+=2)
{
int j=(i+2)%10;
draw_line({a[i],a[i+1]},{a[j],a[j+1]},{255,255,255});
}

for(int i=0;i<10;i+=2)
Transform_matrix_op({a[i],a[i+1]},0,0,300,0,{200,450},&a[i]);


    //define its length number_vertex*2
    int og_length=10;
    int ar_length=10;
    //copy the vertex data in another array for inside point check
    float copy_a[ar_length];
    for(int i=0; i<ar_length; i++)
    {
        copy_a[i]=a[i];
    }

    //initialize triangles that will be formed
    float **tris;
    tris=new float*[(ar_length/2)-2]; //number of tries generated will be number_of_vertex-2
    for(int i=0; i<(ar_length/2)-2; i++)
    {
        tris[i]=new float[6];
    }

    split_concave_poly_into_tris(&a[0],&ar_length,tris);
    getch();
    closegraph();
    return 0;
}

void split_concave_poly_into_tris(float *a,int *ar_length,float *tris[])
{
    int og_length=*ar_length;
    //copy the vertex data in another array for inside point check
    float copy_a[*ar_length];
    for(int i=0; i<*ar_length; i++)
    {
        copy_a[i]=a[i];
    }

    int tri_counter=0;

    while(true)
    {
    for(int i=0; i<*ar_length; i+=2)
    {
        //indexing A,B,C
        int ind_a,ind_b,ind_c;
        ind_a=(i-2);
        if(ind_a<0)
            ind_a=*ar_length-2;
        ind_b=i;
        ind_c=(i+2)%*ar_length;
        //check orientaion of ABC, ear?
        bool clockW=is_clock_wise({a[ind_a],a[ind_a+1]}, {a[ind_b],a[ind_b+1]}, {a[ind_c],a[ind_c+1]});
        //std::cout<<"vert"<<ind_b<<" clock = "<<clockW<<"\n";
        //if ear then check if any points of the og convex inside it
        if(clockW)
        {
            bool inside=false;
            for(int p=0;p<og_length;p+=2)
            {
                inside|=is_point_inside_triangle({a[ind_a],a[ind_a+1]}, {a[ind_b],a[ind_b+1]}, {a[ind_c],a[ind_c+1]},{copy_a[p],copy_a[p+1]});
            }
            if(!inside)//its an ear
            {
                //form the triangle
                tris[tri_counter][0]=a[ind_a];
                tris[tri_counter][1]=a[ind_a+1];
                tris[tri_counter][2]=a[ind_b];
                tris[tri_counter][3]=a[ind_b+1];
                tris[tri_counter][4]=a[ind_c];
                tris[tri_counter][5]=a[ind_c+1];
                tri_counter++;
                //clip it
                adjust_array(a,ar_length,ind_b);
                break;
            }
        }

    }
    if(*ar_length<=2*2)
    break;
    }

    for(int i=0;i<tri_counter;i++)
    {
    for(int j=0;j<6;j+=2)
    {
        int k=(j+2)%6;
        draw_line({tris[i][j],tris[i][j+1]},{tris[i][k],tris[i][k+1]},{255,255,0});
    }
    }
}

void adjust_array(float *a,int* array_length,int drop_index)
{
    *array_length-=2;
    for(int i=drop_index; i<*array_length; i++)
    {
        a[i]=a[i+2];
        a[i+1]=a[i+3];
    }
}




bool is_point_inside_triangle(struct VEC2 A,struct VEC2 B,struct VEC2 C,struct VEC2 P)
{
    int O=is_clock_wise(A,B,P)+is_clock_wise(B,C,P)+is_clock_wise(C,A,P);
    if(O%3==0)
        return true;
    else
        return false;
}


bool is_clock_wise(struct VEC2 A,struct VEC2 B,struct VEC2 C)
{
    struct VEC2 AB= {B.X_Pos-A.X_Pos,B.Y_Pos-A.Y_Pos};
    struct VEC2 BC= {C.X_Pos-B.X_Pos, C.Y_Pos-B.Y_Pos};

    float ABcrossBC=AB.X_Pos * BC.Y_Pos - AB.Y_Pos * BC.X_Pos;

    if(ABcrossBC<0)
        return true;
    else
        return false;
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
