package kitbot;

public class GameStartTest {
	public static void main( String[] args ) {
		
		BotClient botclient = new BotClient("18.150.7.174:6667","mT82Qi240y",false);
		
		while( !botclient.gameStarted() ) {
		}
		System.out.println("***GAME STARTED***");
		System.out.println("MAP --> " + botclient.getMap());
		
		botclient.close();
	}
}
