from customer.apollo.sim.scripts import customer_interface
from simian.public import customer_stack_server

if __name__ == "__main__":
    customer_stack_server.start_server(customer_interface.ApolloInterface)
