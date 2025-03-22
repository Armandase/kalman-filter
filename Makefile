MAKEFLAGS += -j

CXX		=	c++
NAME 	= 	ft_kalman
SRC 	=	main.cpp UdpClient.cpp

HEADER	=	inc/main.hpp inc/UdpClient.h
OBJS	=	$(addprefix obj/, $(SRC:.cpp=.o))
CXXFLAGS=	-Wall -Wextra -Werror --std=c++17 -g

all		:	${NAME} 

${NAME}	:	${OBJS} ${HEADER}
			${CXX} -o ${NAME} ${OBJS}

obj/%.o: src/%.cpp $(HEADER)
	@mkdir -p $(@D)
	${CXX} ${CXXFLAGS} -c $< -o $@

fclean: clean
	rm -f $(NAME)

clean:
	rm -rf obj/

re: fclean
	@make all

.PHONY: all fclean clean re
