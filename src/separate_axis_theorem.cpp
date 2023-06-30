#include<iostream>
#include<graphics.h>
#include<time.h>
#include<conio.h>
#include<math.h>
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

class POLY_COLLIDER
{
public:
    bool dynamic=false;
    float *verts;
    int verts_size;
    int m_id;
    bool in_collision;
    float angle=-90;
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



    void update( class POLY_COLLIDER* colliders[])
    {


        char inp=handleEvent();
        //angle update
        this->angle+=(inp=='A')?5:(inp=='D')?-5:0;
        this->forward_vec={cos((3.1415*angle)/180.0),-sin((3.1415*angle)/180.0)};

        if(this->m_id==0)
        for(int i=0; i<this->verts_size; i+=2)
        {
            //rotation



            //translation
            float x_mov=(inp=='W')?3*this->forward_vec.X_Pos:(inp=='S')?-3*this->forward_vec.X_Pos:0;
            float y_mov=(inp=='W')?3*this->forward_vec.Y_Pos:(inp=='S')?-3*this->forward_vec.Y_Pos:0;

            this->verts[i]+=x_mov;
            this->verts[i+1]+=y_mov;

            this->m_pos.X_Pos+=x_mov;
            this->m_pos.Y_Pos+=y_mov;

        }
        bool col=this->check_collison_sat(colliders);
        if(col)
        this->render({255,0,0});
        else
        this->render({255,255,255});
       // std::cout<<"colision="<<col<<"\n";
    };





    bool check_collison_sat(class POLY_COLLIDER* colliders[])
    {
        //for each poly
        class POLY_COLLIDER* shape1=colliders[m_id];
        class POLY_COLLIDER* shape2=colliders[1-m_id];
        float overlap=100000000;
        for(int i=0; i<2; i++)
        {

            if(i==1)
            {
                shape1=colliders[1-m_id];
                shape2=colliders[m_id];
            }

            for(int a=0; a<shape1->verts_size; a+=2)
            {
                int b=(a+2)%shape1->verts_size;
                float delY=shape1->verts[a+1]-shape1->verts[b+1];
                float delX=shape1->verts[a]-shape1->verts[b];
                struct VEC2 mid_point_of_axis= {(shape1->verts[a]+shape1->verts[b])/2.0, (shape1->verts[a]-shape1->verts[b])/2.0};
                struct VEC2 axis_n_to_e= {-delY,delX};
                // std::cout<<"(nx,ny)="<<axis_n_to_e.X_Pos<<","<<axis_n_to_e.Y_Pos<<"\n";
                //for points in shape1
                float max_ov1=-10000000;
                float min_ov1=10000000;
                for(int j=0; j<shape1->verts_size; j+=2)
                {

                    //dot product btn norm vector and every point of shape 1
                   // float q=((shape1->verts[j]-mid_point_of_axis.X_Pos)*axis_n_to_e.X_Pos)+((shape1->verts[j+1]-mid_point_of_axis.Y_Pos)*axis_n_to_e.Y_Pos);
                    float q=(shape1->verts[j]*axis_n_to_e.X_Pos)+(shape1->verts[j+1]*axis_n_to_e.Y_Pos);
                    min_ov1=std::min(min_ov1,q);
                    max_ov1=std::max(max_ov1,q);
                }

                //for points in shape1
                float max_ov2=-10000000;
                float min_ov2=10000000;
                for(int j=0; j<shape2->verts_size; j+=2)
                {
                    //dot product btn norm vector and every point of shape 1

                    //float q=((shape2->verts[j]-mid_point_of_axis.X_Pos)*axis_n_to_e.X_Pos)+((shape2->verts[j+1]-mid_point_of_axis.Y_Pos)*axis_n_to_e.Y_Pos);
                    float q=(shape2->verts[j]*axis_n_to_e.X_Pos)+(shape2->verts[j+1]*axis_n_to_e.Y_Pos);
                    min_ov2=std::min(min_ov2,q);
                    max_ov2=std::max(max_ov2,q);
                }

                overlap=std::min(std::min(max_ov1,max_ov2)-std::max(min_ov1,min_ov2),overlap);

                if(!(max_ov2>=min_ov1 && max_ov1>=min_ov2))
                {
                    //std::cout<<"max ov1 = "<<max_ov1<<", min ov1 = "<<min_ov1<<"\n"<<"max ov2 = "<<max_ov2<<", min ov2 = "<<min_ov2<<"\n";
                    return false;
                }

            }

        }

        struct VEC2 d={shape2->m_pos.X_Pos-shape1->m_pos.X_Pos, shape2->m_pos.Y_Pos-shape1->m_pos.Y_Pos};
        float s=sqrt(d.X_Pos*d.X_Pos+d.Y_Pos*d.Y_Pos);

        for(int v=0;v<shape1->verts_size;v+=2)
        {
        shape1->verts[v]-=0.02*(overlap*d.X_Pos)/s;
        shape1->verts[v+1]-=0.02*(overlap*d.Y_Pos)/s;
        }
        shape1->m_pos.X_Pos-=0.02*(overlap*d.X_Pos)/s;
        shape1->m_pos.Y_Pos-=0.02*(overlap*d.Y_Pos)/s;

        return true;
    };



    void render(struct VEC3 colors)
    {
    for(int i=0;i<this->verts_size;i+=2)
    {
        int j=(i+2)%this->verts_size;
        draw_line({this->verts[i],this->verts[i+1]},{this->verts[j],this->verts[j+1]},colors);
    }
    if(this->dynamic)
    draw_line({this->verts[4],this->verts[5]},{this->verts[4]+10*this->forward_vec.X_Pos,this->verts[5]+10*this->forward_vec.Y_Pos},colors);
    };
};



int main()
{
    int gd = DETECT, gm;
    //initgraph(&gd, &gm, "C:\\TC\\BGI");
    initwindow(700, 700);
    srand(time(0));


    class POLY_COLLIDER* colliders[100];
    float triangle_verticies[6]= {400,1,
                                  500,1,
                                  450,100
                                 };

    class POLY_COLLIDER* triangle=new POLY_COLLIDER(6,&triangle_verticies[0],colliders,0,{450,50});;;

    float rect_verticies[8]= {1,70,
                              100,70,
                              100,170,
                              1,170
                             };

    class POLY_COLLIDER* rect=new POLY_COLLIDER(8,&rect_verticies[0],colliders,1,{50,120});;

    colliders[0]->dynamic=true;
    //time_loop
    while(true)
    {

        swapbuffers();
        cleardevice();
        for(int i=1; i>=0; i--)
            colliders[i]->update(colliders);

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
        switch (keyCode)
        {
        case 27: // ASCII code for ESC key
            return '`';
            break;
        case 13: // ASCII code for Enter key
            std::cout<<"\N";
            return '\n';
            break;
        case 87: // ASCII code for Enter key
            {
            std::cout<<"W";
            return 'W';
            break;
            }

        case 65: // ASCII code for Enter key
            {
              std::cout<<"A";
            return 'A';
            break;
            }

        case 83: // ASCII code for Enter key
            {
             std::cout<<"S";
            return 'S';
            break;
            }

        case 68: // ASCII code for Enter key
            {
             std::cout<<"D";
            return 'D';
            break;
            }

        default:
            return '.';
      }
}

