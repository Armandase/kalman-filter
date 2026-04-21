NAME = ft_kalman

CXX = c++
CXXFLAGS = -Wall -Wextra -Werror -g -std=c++17

SRC_DIR = src
INC_DIR = inc
OBJ_DIR = obj

SRCS = main.cpp \
       utils.cpp \
       UdpClient.cpp \
       DataStorage.cpp \
       KalmanFilter.cpp

OBJS = $(addprefix $(OBJ_DIR)/, $(SRCS:.cpp=.o))
DEPS = $(OBJS:.o=.d)

EIGEN_DIR = eigen
EIGEN_REPO = https://gitlab.com/libeigen/eigen.git
EIGEN_TAG = 3.4.0

INCLUDES = -I$(INC_DIR) -I$(EIGEN_DIR)

all: $(EIGEN_DIR) $(NAME)

$(EIGEN_DIR):
	git clone --branch $(EIGEN_TAG) --depth 1 $(EIGEN_REPO) $(EIGEN_DIR)

$(NAME): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $(NAME)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -MMD -MP -c $< -o $@

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

clean:
	rm -rf $(OBJ_DIR)

fclean: clean
	rm -f $(NAME)
	# rm -rf $(EIGEN_DIR) # Décommenter pour supprimer la bibliothèque Eigen

re: fclean all

-include $(DEPS)

.PHONY: all clean fclean re
