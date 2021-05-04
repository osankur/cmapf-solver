#include <InstanceLoader.hpp>
#include <iostream>

namespace instance {

	XMLInstanceLoader::XMLInstanceLoader(const std::string& filepath, const std::string& location) : m_filepath(filepath), m_location(location) {}

	void XMLInstanceLoader::load()
	{
        std::ifstream infile(m_filepath);

        std::string line;
        std::string type;
        std::string sMovFile;
        std::string sComFile;

        std::getline(infile, line);
        std::istringstream iss1(line);
        iss1 >> type >> sMovFile;
        sMovFile = m_location + sMovFile;
        std::getline(infile, line);
        std::istringstream iss2(line);
        iss2 >> type >> sComFile;
        sComFile = m_location + sComFile;

        char* movFile = sMovFile.data();
        char* comFile = sComFile.data();

        rapidxml::file<> xmlFile(movFile);
        rapidxml::xml_document<> movDoc;
        movDoc.parse<0>(xmlFile.data());

        const rapidxml::xml_node<>* movGraph = movDoc.first_node("graphml")->first_node("graph");

        // Creating the nodes
        for (const rapidxml::xml_node<>* node = movGraph->first_node("node"); node != NULL; node = node->next_sibling())
        {
            rapidxml::xml_attribute<>* attr = node->first_attribute("id");
            if (attr == NULL)
                continue;
            std::string id(attr->value());
            id.erase(0, 1);
            const rapidxml::xml_node<>* data = node->first_node("data");
            float x = std::stof(data->value());
            data = data->next_sibling();
            float y = std::stof(data->value());
            const XMLNode n(id, x, y);

            m_nodes.push_back(n);
            Node graphNode = (Node)std::stoi(id.c_str());
            m_instance.graph().movement().addNode(graphNode);
            m_instance.graph().movement().addEdge(graphNode, graphNode);
            m_instance.graph().communication().addNode(graphNode);
            m_instance.graph().communication().addEdge(graphNode, graphNode);
        }

        // Creating the movement edges
        for (const rapidxml::xml_node<>* edge = movGraph->first_node("edge"); edge != NULL; edge = edge->next_sibling())
        {
            rapidxml::xml_attribute<>* attr = edge->first_attribute("source");
            if (attr == NULL)
                continue;
            Node source = (Node)std::stoi(attr->value() + 1);
            attr = attr->next_attribute("target");
            Node target = (Node)std::stoi(attr->value() + 1);
            m_instance.graph().movement().addEdge(source, target);
            m_instance.graph().movement().addEdge(target, source);
        }

        xmlFile = rapidxml::file<>(comFile);
        rapidxml::xml_document<> comDoc;
        comDoc.parse<0>(xmlFile.data());

        rapidxml::xml_node<>* comGraph = comDoc.first_node("graphml")->first_node("graph");

        // Creating the communication edges
        for (rapidxml::xml_node<>* edge = comGraph->first_node("edge"); edge != NULL; edge = edge->next_sibling())
        {
            rapidxml::xml_attribute<>* attr = edge->first_attribute("source");
            if (attr == NULL)
                continue;
            Node source = (Node)std::stoi(attr->value() + 1);
            attr = attr->next_attribute("target");
            Node target = (Node)std::stoi(attr->value() + 1);
            m_instance.graph().communication().addEdge(source, target);
            m_instance.graph().communication().addEdge(target, source);
        }

        std::string startConf;
        std::getline(infile, startConf);

        size_t startI = 0;
        auto endI = startConf.find(" ");

        // Creating the start configuration
        while (endI != std::string::npos)
        {
            std::string s = startConf.substr(startI, endI - startI);
            if (s != "start")
                m_instance.start().pushBack(std::stoi(s));
            startI = endI + 1;
            endI = startConf.find(" ", startI);
        }

        std::string goalConf;
        std::getline(infile, goalConf);

        startI = 0;
        endI = goalConf.find(" ");

        // Creating the goal configuration
        while (endI != std::string::npos)
        {
            std::string s = goalConf.substr(startI, endI - startI);
            if (s != "goal")
                m_instance.goal().pushBack(std::stoi(s));
            startI = endI + 1;
            endI = goalConf.find(" ", startI);
        }

        m_instance.setNbAgents(m_instance.start().size());

	}

}