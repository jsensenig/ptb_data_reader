#include <string.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include "pugixml.hpp"
 
using namespace pugi;
using namespace std;

const char* node_types[] =
{
"null", "document", "element", "pcdata", "cdata", "comment", "pi", "declaration", "MyBeerJournal", "Brewery"
};

// tag::impl[]
struct simple_walker: pugi::xml_tree_walker
{
    virtual bool for_each(pugi::xml_node& node)
    {
        for (int i = 0; i < depth(); ++i) std::cout << "  "; // indentation
	

        std::cout << node_types[node.type()] << " ("<< node.type()<< ") : name='" << node.name() << "', value='" << node.child_value() << "'\n";

        for (pugi::xml_attribute attr = node.first_attribute(); attr; attr = attr.next_attribute())
        {
            std::cout << " " << attr.name() << "=" << attr.value();
        }

        std::cout << std::endl;
        return true; // continue traversal
    }
};
// end::impl[]


int main(int argv, char**argc) {

  cout << "Starting" << endl;

  //  ifstream theFile ("config.xml");
  // Read the file into a vector
  //vector<char> buffer((istreambuf_iterator<char>(theFile)), istreambuf_iterator<char>());
  //buffer.push_back('\0');	


  cout << "Start with the XML shenanigans" << endl;
  
  xml_document doc;
  xml_parse_result result = doc.load_file("config.xml");  
  cout << "Let's take a walk..." << endl; 
  simple_walker walker;
  doc.traverse(walker);

  return 0;
}
