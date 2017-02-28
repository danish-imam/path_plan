 #include "opencv2/highgui/highgui.hpp"
 #include "opencv2/imgproc/imgproc.hpp"
 #include <iostream>
 #include <stdio.h>
 #include <stdlib.h>
 #include <vector>
 #include <math.h>
 #include <limits.h>

 using namespace cv;
 using namespace std;

int **graph_array;
 Mat src; Mat src_gray;
 int thresh = 100;
 int max_thresh = 255;
  unsigned int map_graph;

vector<Point> node; 
 /// Function header
void thresh_callback(int, void* );
 void node_gen();


//************************************************************************************************************************
// A C / C++ program for Dijkstra's single source shortest path algorithm.
// The program is for adjacency matrix representation of the graph

  
// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int minDistance(int dist[], bool sptSet[])                                     //``````````````````````````````
{
   // Initialize min value
   int min = INT_MAX, min_index;
  
   for (int v = 0; v < node.size(); v++)
     if (sptSet[v] == false && dist[v] <= min)
         {min = dist[v]; min_index = v;}
  
   return min_index;
 // return 0;
}

void printPath(int parent[], int j)
{
    // Base Case : If j is source
    if (parent[j]==-1)
        return;
 
    printPath(parent, parent[j]);
 
    printf("%d ", j);
}
  
// A utility function to print the constructed distance array
int printSolution(int dist[], int parent[])
{
    int init = 0;
    printf("Vertex\t  Distance\tPath");
    for (int i = 1; i < node.size(); i++)
    {
        printf("\n%d -> %d \t\t %d\t\t%d ", init, i, dist[i], init);
        printPath(parent, i);
    }
}
  
// Funtion that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
void dijkstra()
{
     int dist[node.size()];     // The output array.  dist[i] will hold the shortest
                      // distance from src to i
  
     bool sptSet[node.size()]; // sptSet[i] will true if vertex i is included in shortest
                     // path tree or shortest distance from src to i is finalized
  

    //parent array
    int parent[node.size()];

    
    
     // Initialize all distances as INFINITE and stpSet[] as false
     for (int i = 0; i < node.size(); i++)
        dist[i] = INT_MAX, sptSet[i] = false, parent[0] = -1;
  
     // Distance of source vertex from itself is always 0
     dist[0] = 0;
  
     // Find shortest path for all vertices
     for (int count = 0; count < node.size()-1; count++)
     {
       // Pick the minimum distance vertex from the set of vertices not
       // yet processed. u is always equal to src in first iteration.
       int u = minDistance(dist, sptSet);
       /*if(u>node.size() || u<0)
       {cout << 'a' << endl;exit(0);}*/
       // Mark the picked vertex as processed
       sptSet[u] = true;                                                     //`````````````````````
  
       // Update dist value of the adjacent vertices of the picked vertex.
       for (int v = 0; v < node.size(); v++)
  
         // Update dist[v] only if is not in sptSet, there is an edge from 
         // u to v, and total weight of path from src to  v through u is 
         // smaller than current value of dist[v]
         if (!sptSet[v] && graph_array[u][v] /*&& dist[u] != INT_MAX*/ 
                                       && dist[u]+graph_array[u][v] < dist[v])      //````````````````````````````
            {
              parent[v]=u;
              dist[v] = dist[u] + graph_array[u][v];
            }
     }
  
     // print the constructed distance array
     printSolution(dist, parent);
}                                                                                 //`````````````````````````````
  
// driver program to test above function
/*int main()
{
   /* Let us create the example graph discussed above */
  /* int graph[V][V] = {{0, 4, 0, 0, 0, 0, 0, 8, 0},
                      {4, 0, 8, 0, 0, 0, 0, 11, 0},
                      {0, 8, 0, 7, 0, 4, 0, 0, 2},
                      {0, 0, 7, 0, 9, 14, 0, 0, 0},
                      {0, 0, 0, 9, 0, 10, 0, 0, 0},
                      {0, 0, 4, 14, 10, 0, 2, 0, 0},
                      {0, 0, 0, 0, 0, 2, 0, 1, 6},
                      {8, 11, 0, 0, 0, 0, 1, 0, 7},
                      {0, 0, 2, 0, 0, 0, 6, 7, 0}
                     };
  
    dijkstra(graph, 0);
  
    return 0;
}*/

//**************************************************************************************************************************************
 int isValid(int i,int j)//to include edges
{
	if((i<0)||(j<0)||(i>=src.rows)||(j>=src.cols))
		return 0;
	else
		return 1;
}

int node_con(Point a, Point b)//checking if these nodes are connected or not i.e., if obstacle or not
{
   int flag = 1;
   int init = (a.x<b.x)? a.x:b.x;
   int X,Y;
   float m, c;//slope and y-intercept
   if(a.x!=b.x)
   {
    m = (a.y-b.y)/(a.x-b.x);
    c = b.y-m*b.x;
    for(int i=0;i<abs(a.x-b.x);i++)
    {
        X=init + i;
        Y =(int)(X*m + c);
        if(src_gray.at<uchar>(X, Y) == 255)
        {return 0;}
    }
   }
   return flag;
}


/** @function main */
int main()
 {
   /// Load source image and convert it to gray
   src = imread( "try.png", 1 );

   /// Convert image to gray and blur it
   cvtColor( src, src_gray, CV_BGR2GRAY );
   blur( src_gray, src_gray, Size(3,3) );//smoothing the image

   /// Create Window
   namedWindow( "Source", CV_WINDOW_AUTOSIZE );
   imshow( "Source", src );

   createTrackbar( " Threshold:", "Source", &thresh, max_thresh, thresh_callback );
   thresh_callback( 0, 0 );//source:docs opencv ,not taught in wws

   
    dijkstra();                                                     //````````````````````````

//printing of adjacency matrix
  /* for(int i=0;i<node.size();i++)
   {
    for(int j=0;j<node.size();j++)
    cout << graph_array[i][j] << "  ";
    cout << endl;
   }*/


   waitKey(0);
   return(0);
 }

 /** @function thresh_callback */
 void thresh_callback(int, void* )
 {
   Mat src_copy = src.clone();
   Mat threshold_output;
   vector<vector<Point> > contours;
   vector<Vec4i> hierarchy;

   /// Detect edges using Threshold
   threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );

   /// Find contours
   findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE /*, Point(0, 0)*/ );

   /// Find the convex hull object for each contour
   vector<vector<Point> >hull( contours.size());
   for( int i = 0; i < contours.size(); i++ )
      {  convexHull( Mat(contours[i]), hull[i], false ); }

   /// Draw contours + hull results
   Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );//declaring and initialising all black
                //inplace of threshold_output.size() u can use threshold_output.rows, threshold_output.cols
   for( int i = 0; i< contours.size(); i++ )
      {
          Scalar color = Scalar(255, 255, 255);//linesdrawn will be white coloured
        //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        //drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        drawContours( drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        //printf("%ld\n", contours.size());//number of polygon
        for( int j=0;j<hull[i].size();j++)
        {
             //cout << hull[i].size() << endl;
             //cout << hull[i][j] << endl;
             if(src_gray.at<uchar>(i, j) == 0)
             node.push_back(hull[i][j]);//storing the edges as nodes
        } 
        //cout << endl;
      }
    
   imwrite("convexhull.jpg",drawing);

   node_gen();//generation of nodes for graph
   /// Show in a window
   namedWindow( "Hull demo", CV_WINDOW_AUTOSIZE );
   imshow( "Hull demo", drawing );
 }

void node_gen()
{
  int i, j;

  //taking nodes as edges
  for(i=0;i< src.rows; i+=10)
  {
    if(src_gray.at<uchar>(i,0) == 0)
    node.push_back(Point(i, 0));

    if(src_gray.at<uchar>(i,src.cols-1) == 0)
    node.push_back(Point(i, src.cols-1));
  }
  for(i=0;i< src.cols; i+=10)
  {
    if(src_gray.at<uchar>(0, i) == 0)
    node.push_back(Point(0, i));

    if(src_gray.at<uchar>(src.rows-1, i) == 0)
    node.push_back(Point(src.rows-1, i));
  }

  int r=0;
  int current_size = node.size();
  //cout << current_size << endl;
  for(i=0; i<current_size;i++)
  {
    r=0;
    int x=node[i].x;
    int y=node[i].y;
    while(r<abs(src.rows-x) || r<abs(src.cols-y))
    {
      r+=10;
      if(isValid((x+r),(y+r)))
      node.push_back(Point(x+r, y+r));
      if(isValid((x+r),(y-r)))
      node.push_back(Point(x+r, y-r));
      if(isValid((x-r),(y+r)))
      node.push_back(Point(x-r, y+r));
      if(isValid((x-r),(y-r)))
      node.push_back(Point(x-r, y-r));
    }
  }
    node.insert(node.begin(), Point(0, 0));
  node.push_back(Point(src.rows, src.cols));

  current_size = node.size();

  //generation of graph_array
  graph_array  = (int **)malloc(sizeof(int *) * current_size);
  graph_array[0] = (int *)malloc(sizeof(int) * current_size * current_size);
  for(i = 0; i < current_size; i++)
        graph_array[i] = (*graph_array + current_size * i);
  //I have got this idea from geekforgeeks this dynamic array
  

  unsigned int c=0;
  //cout << current_size << endl;
  //cout << src.rows*src.cols << endl;
  for(i=0;i<current_size;i++)
  {
    for(j=0;j<current_size ;j++)
    {
      if(i!=j)
      {
        if(node_con(node[i], node[j]))
        {
          graph_array[i][j]= abs(node[i].x - node[j].x) + abs(node[i].y - node[j].y);
          c++;
        }
        else
        graph_array[i][j]=0;
      }
      else 
      graph_array[i][j]=0;
    }//for j
  }//for i
  //cout << c << endl;
}
//    END OF node_gen


