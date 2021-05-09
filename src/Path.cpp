#include <Path.hpp>
#include <string>

void Path::PushBack(Node node)
{
	path_.push_back(node);
}

const Node& Path::GetAtTimeOrLast(size_t time) const
{
	return time < path_.size() ? path_[time] : path_.back();
}

Node& Path::operator[](size_t time)
{
	return path_[time];
}

const Node& Path::operator[](size_t time) const
{
	return path_[time];
}

size_t Path::size() const
{
	return path_.size();
}

// Friends

std::ostream& operator<<(std::ostream& os, const Path& path)
{
	os << "[";
	for (auto it = path.path_.cbegin(); it != path.path_.cend(); ++it)
	{
		os << std::to_string(*it);
		if (std::distance(it, path.path_.cend()) > 1)
			os << ", ";
	}
	os << "]";
	return os;
}