// metro_system.hpp
#pragma once
#include <iostream>
#include <unordered_map>
#include <vector>
#include <string>
#include <queue>
#include <list>
#include <cmath>
#include <climits>
#include <algorithm>
using namespace std;

class Graph_M {
private:
    struct Vertex {
        unordered_map<string, int> nbrs;
    };

    unordered_map<string, Vertex> vtces;

    struct DijkstraPair {
        string vname;
        string psf;
        int cost;

        bool operator<(const DijkstraPair& other) const {
            return other.cost < cost;  // For min heap
        }
    };

    struct Pair {
        string vname;
        string psf;
        int min_dis;
        int min_time;

        Pair() : min_dis(0), min_time(0) {}
    };

public:
    Graph_M() = default;

    int numVertex() const {
        return vtces.size();
    }

    bool containsVertex(const string& vname) const {
        return vtces.find(vname) != vtces.end();
    }

    void addVertex(const string& vname) {
        if (!containsVertex(vname)) {
            Vertex vtx;
            vtces[vname] = vtx;
        }
    }

    void removeVertex(const string& vname) {
        if (!containsVertex(vname)) return;

        Vertex& vtx = vtces[vname];
        vector<string> keys;
        for (const auto& nbr : vtx.nbrs) {
            keys.push_back(nbr.first);
        }

        for (const string& key : keys) {
            Vertex& nbrVtx = vtces[key];
            nbrVtx.nbrs.erase(vname);
        }

        vtces.erase(vname);
    }

    int numEdges() const {
        int count = 0;
        for (const auto& vtx_pair : vtces) {
            count += vtx_pair.second.nbrs.size();
        }
        return count / 2;
    }

    bool containsEdge(const string& vname1, const string& vname2) const {
        if (!containsVertex(vname1) || !containsVertex(vname2)) {
            return false;
        }
        return vtces.at(vname1).nbrs.find(vname2) != vtces.at(vname1).nbrs.end();
    }

    void addEdge(const string& vname1, const string& vname2, int value) {
        if (!containsVertex(vname1) || !containsVertex(vname2)) {
            return;
        }
        if (containsEdge(vname1, vname2)) {
            return;
        }
        vtces[vname1].nbrs[vname2] = value;
        vtces[vname2].nbrs[vname1] = value;
    }

    void removeEdge(const string& vname1, const string& vname2) {
        if (!containsEdge(vname1, vname2)) {
            return;
        }
        vtces[vname1].nbrs.erase(vname2);
        vtces[vname2].nbrs.erase(vname1);
    }

    void display_Map() const {
        cout << "\n\tDelhi Metro Map" << endl;
        cout << "\t------------------" << endl;
        cout << "----------------------------------------------------\n" << endl;

        for (const auto& vtx_pair : vtces) {
            cout << vtx_pair.first << " =>\n";
            const Vertex& vtx = vtx_pair.second;

            for (const auto& nbr : vtx.nbrs) {
                cout << "\t" << nbr.first;
                if (nbr.first.length() < 16) cout << "\t";
                if (nbr.first.length() < 8) cout << "\t";
                cout << nbr.second << "\n";
            }
            cout << endl;
        }

        cout << "\t------------------" << endl;
        cout << "---------------------------------------------------\n" << endl;
    }

    void display_Stations() const {
        cout << "\n***********************************************************************\n" << endl;
        int i = 1;
        for (const auto& vtx_pair : vtces) {
            cout << i << ". " << vtx_pair.first << endl;
            i++;
        }
        cout << "\n***********************************************************************\n" << endl;
    }

    bool hasPath(const string& vname1, const string& vname2, unordered_map<string, bool>& processed) const {
        if (containsEdge(vname1, vname2)) return true;

        processed[vname1] = true;

        for (const auto& nbr : vtces.at(vname1).nbrs) {
            if (processed.find(nbr.first) == processed.end()) {
                if (hasPath(nbr.first, vname2, processed)) return true;
            }
        }
        return false;
    }

    int dijkstra(const string& src, const string& des, bool isTime) const {
        int val = 0;
        unordered_map<string, DijkstraPair> map;
        priority_queue<DijkstraPair> heap;

        // Initialize
        for (const auto& vtx_pair : vtces) {
            DijkstraPair np;
            np.vname = vtx_pair.first;
            np.cost = INT_MAX;

            if (vtx_pair.first == src) {
                np.cost = 0;
                np.psf = vtx_pair.first;
            }

            heap.push(np);
            map[vtx_pair.first] = np;
        }

        while (!heap.empty()) {
            DijkstraPair rp = heap.top();
            heap.pop();

            if (rp.vname == des) {
                val = rp.cost;
                break;
            }

            map.erase(rp.vname);

            for (const auto& nbr : vtces.at(rp.vname).nbrs) {
                if (map.find(nbr.first) != map.end()) {
                    int oc = map[nbr.first].cost;
                    int nc = isTime ?
                        rp.cost + 120 + (nbr.second * 60 / 40) : // Time in minutes
                        rp.cost + nbr.second;                     // Distance in km

                    if (nc < oc) {
                        DijkstraPair gp = map[nbr.first];
                        gp.psf = rp.psf + " -> " + nbr.first;
                        gp.cost = nc;
                        map[nbr.first] = gp;
                        heap.push(gp);
                    }
                }
            }
        }
        return val;
    }

    string Get_Minimum_Distance(const string& src, const string& dst) const {
        if (!containsVertex(src) || !containsVertex(dst)) {
            return "Invalid stations";
        }

        int min_dist = INT_MAX;
        string path = "";
        unordered_map<string, bool> processed;
        list<Pair> stack;

        Pair sp;
        sp.vname = src;
        sp.psf = src;

        stack.push_front(sp);

        while (!stack.empty()) {
            Pair rp = stack.front();
            stack.pop_front();

            if (processed[rp.vname]) continue;

            processed[rp.vname] = true;

            if (rp.vname == dst) {
                if (rp.min_dis < min_dist) {
                    path = rp.psf;
                    min_dist = rp.min_dis;
                }
                continue;
            }

            for (const auto& nbr : vtces.at(rp.vname).nbrs) {
                if (!processed[nbr.first]) {
                    Pair np;
                    np.vname = nbr.first;
                    np.psf = rp.psf + " -> " + nbr.first;
                    np.min_dis = rp.min_dis + nbr.second;
                    stack.push_front(np);
                }
            }
        }
        return path + "\nDistance: " + to_string(min_dist) + " km";
    }

    string Get_Minimum_Time(const string& src, const string& dst) const {
        if (!containsVertex(src) || !containsVertex(dst)) {
            return "Invalid stations";
        }

        int min_time = INT_MAX;
        string path = "";
        unordered_map<string, bool> processed;
        list<Pair> stack;

        Pair sp;
        sp.vname = src;
        sp.psf = src;

        stack.push_front(sp);

        while (!stack.empty()) {
            Pair rp = stack.front();
            stack.pop_front();

            if (processed[rp.vname]) continue;

            processed[rp.vname] = true;

            if (rp.vname == dst) {
                int total_time = rp.min_time + (get_Interchanges(rp.psf) * 120);
                if (total_time < min_time) {
                    path = rp.psf;
                    min_time = total_time;
                }
                continue;
            }

            for (const auto& nbr : vtces.at(rp.vname).nbrs) {
                if (!processed[nbr.first]) {
                    Pair np;
                    np.vname = nbr.first;
                    np.psf = rp.psf + " -> " + nbr.first;
                    np.min_time = rp.min_time + (nbr.second * 60 / 40); // Time in minutes
                    stack.push_front(np);
                }
            }
        }
        return path + "\nTime: " + to_string(min_time) + " minutes";
    }

    int get_Interchanges(const string& path) const {
        int count = 0;
        size_t pos = path.find('~');
        if (pos == string::npos) return 0;

        char last_line = path[pos + 1];

        while ((pos = path.find('~', pos + 1)) != string::npos) {
            char current_line = path[pos + 1];
            if (current_line != last_line) {
                count++;
                last_line = current_line;
            }
        }
        return count;
    }

    static void Create_Metro_Map(Graph_M& g) {
        g.addVertex("Anarkali");
        g.addVertex("Girja Chowk");
        g.addVertex("Railway Station");
        g.addVertex("Nasir Bagh");
        g.addVertex("Sadar");
        g.addVertex("Nadirabad");
        g.addVertex("PAF Market");
        g.addVertex("Cavalry Ground");
        g.addVertex("Afsha Chowk");
        g.addVertex("Club Road");
        g.addVertex("Malir Cantt");
        g.addVertex("Jinnah Camp");
        g.addVertex("Tufail Road");
        g.addVertex("Nishat Colony");
        g.addVertex("Sports Complex");
        g.addVertex("Dhandi Sarak");

        g.addEdge("Anarkali", "Girja Chowk", 8);
        g.addEdge("Girja Chowk", "Railway Station", 10);
        g.addEdge("Railway Station", "Sadar", 8);
        g.addEdge("Railway Station", "Nasir Bagh", 6);
        g.addEdge("Nasir Bagh", "Nadirabad", 9);
        g.addEdge("Nadirabad", "PAF Market", 7);
        g.addEdge("PAF Market", "Cavalry Ground", 6);
        g.addEdge("Afsha Chowk", "Club Road", 15);
        g.addEdge("Club Road", "Nishat Colony", 6);
        g.addEdge("Nishat Colony", "Nasir Bagh", 7);
        g.addEdge("Nasir Bagh", "Tufail Road", 1);
        g.addEdge("Tufail Road", "Jinnah Camp", 2);
        g.addEdge("Jinnah Camp", "Malir Cantt", 5);
        g.addEdge("Tufail Road", "Sports Complex", 2);
        g.addEdge("Sports Complex", "Dhandi Sarak", 7);
        g.addEdge("Dhandi Sarak", "Cavalry Ground", 8);
        g.addEdge("Nadirabad", "Sadar", 2);
        g.addEdge("Malir Cantt", "Sadar", 2);
        g.addEdge("Malir Cantt", "Afsha Chowk", 3);
    
    }
};

// main function
int main() {
    Graph_M g;
    Graph_M::Create_Metro_Map(g);

    cout << "\n\t\t\t****WELCOME TO THE METRO APP*****" << endl;

    while (true) {
        cout << "\n\t\t\t\t~~LIST OF ACTIONS~~\n\n";
        cout << "1. LIST ALL THE STATIONS IN THE MAP\n";
        cout << "2. SHOW THE METRO MAP\n";
        cout << "3. GET SHORTEST DISTANCE FROM A 'SOURCE' STATION TO 'DESTINATION' STATION\n";
        cout << "4. GET SHORTEST TIME TO REACH FROM A 'SOURCE' STATION TO 'DESTINATION' STATION\n";
        cout << "5. GET SHORTEST PATH (DISTANCE WISE) FROM A 'SOURCE' STATION TO 'DESTINATION' STATION\n";
        cout << "6. GET SHORTEST PATH (TIME WISE) FROM A 'SOURCE' STATION TO 'DESTINATION' STATION\n";
        cout << "7. EXIT THE MENU\n\n";
        cout << "ENTER YOUR CHOICE (1-7): ";

        string input;
        getline(cin, input);
        int choice;
        try {
            choice = stoi(input);
        }
        catch (...) {
            choice = -1;
        }

        if (choice == 7) {
            cout << "\nThank you for using the Metro App!\n";
            break;
        }

        string source, destination;
        switch (choice) {
        case 1:
            g.display_Stations();
            break;
        case 2:
            g.display_Map();
            break;
        case 3:
            cout << "\nEnter source station: ";
            getline(cin, source);
            cout << "Enter destination station: ";
            getline(cin, destination);

            if (!g.containsVertex(source) || !g.containsVertex(destination)) {
                cout << "\nInvalid station(s)!\n";
                break;
            }
            cout << "\nShortest distance: " << g.dijkstra(source, destination, false) << " km\n";
            break;
        case 4:
            cout << "\nEnter source station: ";
            getline(cin, source);
            cout << "Enter destination station: ";
            getline(cin, destination);

            if (!g.containsVertex(source) || !g.containsVertex(destination)) {
                cout << "\nInvalid station(s)!\n";
                break;
            }
            cout << g.Get_Minimum_Time(source, destination) << endl;
            break;
        case 5:
            cout << "\nEnter source station: ";
            getline(cin, source);
            cout << "Enter destination station: ";
            getline(cin, destination);

            if (!g.containsVertex(source) || !g.containsVertex(destination)) {
                cout << "\nInvalid station(s)!\n";
                break;
            }
            cout << g.Get_Minimum_Distance(source, destination) << endl;
            break;
        case 6:
            cout << "\nEnter source station: ";
            getline(cin, source);
            cout << "Enter destination station: ";
            getline(cin, destination);

            if (!g.containsVertex(source) || !g.containsVertex(destination)) {
                cout << "\nInvalid station(s)!\n";
                break;
            }
            cout << "\nMinimum time: " << g.dijkstra(source, destination, true) << " minutes\n";
            break;
        default:
            cout << "\nInvalid choice! Please enter a number between 1 and 7.\n";
        }
    }
    return 0;
}