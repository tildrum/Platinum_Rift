#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <list>
#include <map>
 
#include <limits> // for numeric_limits
 
#include <queue>
#include <utility> // for pair
//#include <iterator>

using namespace std;

 /*
 
    Premier temps : le déplacement. Chaque joueur déplace autant de troupes qu'il le souhaite sur la carte.
    Deuxième temps : l'achat. De nouveaux PODs sont achetés automatiquement si assez de platine est disponible. (20 platine pour achat)
    Troisième temps : la distribution. Chaque joueur reçoit un nombre de barres de Platine fonction du nombre de barres de platine sur les zones conquises.
    Quatrième temps : le combat. Les combats sont déclenchés sur les zones lorsque les deux joueurs ont terminé les 3 premières étapes.
    Cinquième temps : l'affectation. L'appartenance des zones change.
*/

typedef int vertex_t;
typedef double weight_t;
 
const weight_t max_weight = numeric_limits<double>::infinity();
 
struct neighbor {
    
    vertex_t target;
    weight_t weight;
    
    neighbor(vertex_t arg_target)
        : target(arg_target) { }
    neighbor(vertex_t arg_target, weight_t arg_weight)
        : target(arg_target), weight(arg_weight) { }
};
 
typedef vector<vector<neighbor> > adjacency_list_t;
typedef pair<weight_t, vertex_t> weight_vertex_pair_t;

void DijkstraComputePaths(vertex_t source,
                          const adjacency_list_t &adjacency_list, 
                          vector<weight_t> &min_distance,
                          vector<vertex_t> &previous)
{
    int n = adjacency_list.size();
    min_distance.clear();
    min_distance.resize(n, max_weight);
    min_distance[source] = 0;
    previous.clear();
    previous.resize(n, -1);
    // we use greater instead of less to turn max-heap into min-heap
    std::priority_queue<weight_vertex_pair_t,
			std::vector<weight_vertex_pair_t>,
			std::greater<weight_vertex_pair_t> > vertex_queue;
    vertex_queue.push(std::make_pair(min_distance[source], source));
 
    while (!vertex_queue.empty()) 
    {
        weight_t dist = vertex_queue.top().first;
        vertex_t u = vertex_queue.top().second;
        vertex_queue.pop();
 
	// Because we leave old copies of the vertex in the priority queue
	// (with outdated higher distances), we need to ignore it when we come
	// across it again, by checking its distance against the minimum distance
	
	if (dist > min_distance[u])
	    continue;
 
        // Visit each edge exiting u
	const std::vector<neighbor> &neighbors = adjacency_list[u];
	
        for (std::vector<neighbor>::const_iterator neighbor_iter = neighbors.begin();
             neighbor_iter != neighbors.end();
             neighbor_iter++)
        {
            vertex_t v = neighbor_iter->target;
            weight_t weight = neighbor_iter->weight;
            weight_t distance_through_u = dist + weight;
            
    	    if (distance_through_u < min_distance[v]) 
    	    {
    	        min_distance[v] = distance_through_u;
    	        previous[v] = u;
    	        vertex_queue.push(std::make_pair(min_distance[v], v));
    	    }
        }
    }
}
std::list<vertex_t> DijkstraGetShortestPathTo(
    vertex_t vertex, const std::vector<vertex_t> &previous)
{
    std::list<vertex_t> path;
    for ( ; vertex != -1; vertex = previous[vertex])
        path.push_front(vertex);
    return path;
}

struct zone
{
    int zoneCount;
    int linkCount;
    
    int m_id;
    int m_nbrPlatinum;
    
    int s_ownerId; // the player who owns this zone (-1 otherwise)
    int s_podsP0; // player 0's PODs on this zone
    int s_podsP1; // player 1's PODs on this zone
    int s_visible; // 1 if one of your units can see this tile, else 0
    int s_platinum; // the amount of Platinum this zone can provide (0 if hidden by fog)
    float coefNote;
    
    static int compteurZone;
    
    vector<neighbor> voisin;
    
    zone(int zoneId, int platinumSource,int linkCount, int zoneCount ) : m_id(zoneId), m_nbrPlatinum(platinumSource), linkCount(linkCount), zoneCount(zoneCount)
    {
        coefNote=1;
        compteurZone++;
    }
    void getCompteurZone()
    {
        //+ << "il y a actuellement tant de zone " << compteurZone << endl;
    }
    void debug()
    {
        //+ << "voisin size de la zone :  " << m_id << "  : " << voisin[0].target << endl;
    }
    void ajoutVoisin(neighbor & s)
    {
        voisin.push_back(s);   
    }
    void maj(int ownerId, int podsP0, int podsP1, int visible, int platinum)
    {
        s_ownerId = ownerId;
        s_podsP0 = podsP0;
        s_podsP1 = podsP1;
        s_visible = visible;
        s_platinum = platinum;
    }
};

class pod
{
    zone *zonePod;
    
    int myId;
    int m_id;
    
    int place;
    int placeI;
    int nbrVoisin;
    int nbrEnnemieAdj;
    int nbrPlatineNeutralAdj;
    int nbrPlatineAdj;
    
    float meilleurNote;
    priority_queue<float> note;
    
    static int compteurPod;
    
    public :
    
    pod(int id, zone *a) : m_id(id), zonePod(a) { compteurPod++; }
    ~pod() { compteurPod--; }
    void getInstence()
    {
        cerr << " il y actuellement tant de pod : " << compteurPod << endl;
    }
    int getCompteur()
    {
        return compteurPod;
    }
    void debug()
    {
        cerr << "TEST " << m_id << endl;
    }
    void deplacementPod(int place, zone *a, string &deplacement)
    {
        deplacement += "1 ";
        stringstream ss;
        
        ss << zonePod->m_id << " " << placeI << " ";
        string str = ss.str();
        deplacement += str;
        str.clear();
        zonePod->coefNote = 0.1; 
        zonePod = a;
        zonePod->coefNote = 0.1;
        
        while (!note.empty()) { note.pop(); }
    }
    void notation(vector<zone*> &mappy, int myId1, string &deplacement)
    {
        placeI = 0;
        float noteI(0);
        nbrVoisin = zonePod->voisin.size();
        myId = myId1;
        
        for(int j = 0 ; j<nbrVoisin; j++)
        {
            float notetmp;
            place = zonePod->voisin[j].target;
            
            getPlatinNeutralAdjacent(mappy);
            getPlatinEnnemieAdjacent(mappy);
            getPlatinAdjacent(mappy);
            
            if (mappy[place]->s_ownerId != myId)
            {
                notetmp = 0.1 + mappy[place]->s_platinum;
                notetmp += (0.25*(nbrPlatineNeutralAdj + 0.75*nbrEnnemieAdj))/nbrPlatineAdj;
                notetmp *= zonePod->coefNote;
                note.push(notetmp);
                notetmp = 0;
                
                nbrPlatineNeutralAdj = 0;
                nbrEnnemieAdj = 0;
                nbrPlatineAdj = 0;
            }
            
            if(noteI != note.top())
            {
                noteI = note.top();
                placeI = place;
            }
        }
        deplacementPod(placeI, mappy[placeI], deplacement);
    }
    void getPlatinNeutralAdjacent(vector<zone*> &mappy)
    { 
        for(int i=0; i<nbrVoisin; i++)
        {
            if(mappy[zonePod->voisin[i].target]->s_ownerId == -1)
            {
                nbrPlatineNeutralAdj += mappy[zonePod->voisin[i].target]->s_platinum;
            }
        }
    }
    void getPlatinEnnemieAdjacent(vector<zone*> &mappy)
    { 
        for(int i=0; i<nbrVoisin; i++)
        {
            if(mappy[zonePod->voisin[i].target]->s_ownerId != myId && mappy[zonePod->voisin[i].target]->s_ownerId != -1)
            {
                nbrEnnemieAdj += mappy[zonePod->voisin[i].target]->s_platinum;
            }
        }
    }
    void getPlatinAdjacent(vector<zone*> &mappy)
    {   
        for(int i=0; i<nbrVoisin; i++)
        {
            nbrPlatineAdj += mappy[zonePod->voisin[i].target]->s_platinum;   
        }
    }
};

class game
{
    int playerCount; // the amount of players (always 2)
    int myId; // my player ID (0 or 1)
    int zoneCount; // the amount of zones on the map
    int linkCount; // the amount of links between all zones
    int maBase;
    int ennemieBase;
    int n,t;
    
    int myPlatinum; // your available Platinum
    
    int zId; // this zone's ID
    int ownerId; // the player who owns this zone (-1 otherwise)
    int podsP0; // player 0's PODs on this zone
    int podsP1; // player 1's PODs on this zone
    int visible; // 1 if one of your units can see this tile, else 0
    int platinum; // the amount of Platinum this zone can provide (0 if hidden by fog)
    
    //string deplacement;
    
    vector<int> pathAttaque;
    
    vector<zone*> mappy;
    adjacency_list_t adjacency_list;
    vector<pod*> Pod;
    
    friend class pod;
    
    public :
    
    game()
    {
        cin >> playerCount >> myId >> zoneCount >> linkCount; cin.ignore();
        //+ << " MY ID : " << myId << endl;
        adjacency_list.resize(zoneCount);
        
        t=0;
        n=0;
        
        for (int i = 0; i < zoneCount; i++) {
            int zoneId; // this zone's ID (between 0 and zoneCount-1)
            int platinumSource; // Because of the fog, will always be 0
            
            cin >> zoneId >> platinumSource; cin.ignore();
            
            mappy.push_back( new zone (zoneId, platinumSource, linkCount, zoneCount));
        }
        for (int i = 0; i < linkCount; i++) {
            
            int zone1;
            int zone2;
            cin >> zone1 >> zone2; cin.ignore();
            
            neighbor a(zone2), b(zone1);  // on Stocke les voisins
            mappy[zone1]->ajoutVoisin(a);    
            mappy[zone2]->ajoutVoisin(b);
            
            adjacency_list[zone1].push_back(neighbor(zone2, 1));
            adjacency_list[zone2].push_back(neighbor(zone1, 1));
        }
    }
    void debug()
    {
         //+ << " ma base : " << maBase << " Ennemie base ; " << ennemieBase << endl;
         Pod[0]->getInstence();
         mappy[0]->getCompteurZone();
    }
    void init()
    {
        cin >> myPlatinum; cin.ignore();

        for (int i = 0; i < zoneCount; i++) 
        {    
            cin >> zId >> ownerId >> podsP0 >> podsP1 >> visible >> platinum; cin.ignore(); // platine et visible peuvent viré ? 
           // cerr << " Sortie du jeu : " << zId << " : " << ownerId << " : " << podsP0 << " : " << podsP1 << " : " << visible << " : " << platinum << endl;
            
            mappy[i]->maj(ownerId, podsP0, podsP1, visible, platinum);
            
            if(ownerId == myId)
            {
                maBase = zId;
                //+ << " MA base : " << maBase << endl;
                if(myId == 1)
                {
                    for(int i=0; i<podsP1; i++)
                    {
                        Pod.push_back(new pod( i, mappy[maBase]));
                    }
                }
                else if(myId == 0)
                {
                    for(int i=0; i<podsP0; i++)
                    {
                        Pod.push_back(new pod( i, mappy[maBase]));
                    }
                }
            }
            else if (ownerId != myId && ownerId != -1)
            {
                ennemieBase = zId;
            }
        }
        t = Pod[0]->getCompteur();
        dijkStraAlgo();
    }
    void dijkStraAlgo()
    {
        vector<weight_t> min_distance;
        vector<vertex_t> previous;
        
        DijkstraComputePaths(maBase, adjacency_list, min_distance, previous);
        
        list<vertex_t> path = DijkstraGetShortestPathTo(ennemieBase, previous);
        
        pathAttaque.resize(path.size());
        
        copy(path.begin(), path.end(), pathAttaque.begin());
    }
    
    void maj()
    {
        cerr << " TEST 5" << endl;
        
        if(n>=1)
        {
            cin >> myPlatinum; cin.ignore();
            
            for (int i = 0; i < zoneCount; i++) 
            {
                cin >> zId >> ownerId >> podsP0 >> podsP1 >> visible >> platinum; cin.ignore(); 
                mappy[i]->maj(ownerId, podsP0, podsP1, visible, platinum);
                //cerr << " Sortie du jeu MAJ : " << zId << " : " << ownerId << " : " << podsP0 << " : " << podsP1 << " : " << visible << " : " << platinum << endl;
            }
        }
        else {
            
            init();
            n++;
        }
        //+ << "TEST MAJ  : " << n <<  endl;
    }
    void move(string &deplacement)
    {
        cerr << " TEST 1" << endl;
        int i = 0;
        
        for(vector<pod*>::iterator it = Pod.begin(); it != Pod.end(); it++)
        {
            cerr << " TEST segm : " << Pod.size() << " avec iun I ; " << i  << endl;
            
            (*it)->debug();
            //cerr << "TEST DEPLACEMENT : " << deplacement << endl;
            (*it)->notation(mappy, myId, deplacement);
            //cerr << "TEST DEPLACEMENT : " << deplacement << endl;
            i++;
        }
    }
    void achatPod()
    {
        cerr << " TEST 2" << endl;
        do
        {
            ////+ << "Platinum avant achat : " << myPlatinum << endl;
            if(myPlatinum>19)
            {
             //   //+ << "ACHART POD : " << endl;
                Pod.push_back(new pod(t, mappy[maBase]));
                myPlatinum -= 20;
                t++;
            }
        }while(myPlatinum >= 20);
    }
    void combat()
    {
        cerr << " TEST 4" << endl;
    }
};

int pod::compteurPod=0;
int zone::compteurZone=0;

int main()
{
    
    game game1;
    
    while (1) {
        
        string deplacement;
        
        game1.maj();            // TEST 5
        game1.move(deplacement);    // TEST 1
        
        //cerr << "TEST DEPLACEMENT : " << deplacement.size() << endl;
        game1.achatPod();           // TEST 2
        //game1.majPlatinum();        // TEST 3
        game1.combat();         // TEST 4
        
        cout << deplacement << endl;
        cout << "WAIT" << endl;
        //game1.debug();
        //deplacement.clear();
    }
}
