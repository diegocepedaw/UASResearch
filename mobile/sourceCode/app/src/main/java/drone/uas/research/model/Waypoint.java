package drone.uas.research.model;

import com.google.android.gms.maps.model.LatLng;

/**
 * Immutable abstract data type. Represents a labeled geographical location.
 * Created by stanc on 4/24/16.
 */
public class Waypoint {
    private String mName;
    private LatLng mCoords;

    /**
     * Constructor. Pass in the label for the desired coordinates.
     * @param mName
     * @param mCoords
     */
    public Waypoint(String mName, LatLng mCoords){
        this.mName = mName;
        this.mCoords = mCoords;
    }

    /**
     * Returns the name of Waypoint
     * @return String representation of the label.
     */
    public String getName(){
        return mName;
    }

    /**
     * Returns the coordinates of the waypoint
     * @return Waypoint representation of the geographical location.
     */
    public LatLng getCoords(){
        return mCoords;
    }

    /**
     * Returns true if parameter is a Waypoint object with the same name and coordinates.
     * @param o Object to compare against.
     * @return
     */
    @Override
    public boolean equals(Object o) {
        if(o instanceof Waypoint){
            Waypoint temp = (Waypoint)o;
            return mName.equals(temp.mName) && mCoords.equals(temp.mCoords);
        }
        return false;
    }

    /**
     * Returns the hashcode representation. Characterized by the name and coordinates.
     * @return
     */
    @Override
    public int hashCode() {
        int i = 0;
        i += 17 * mName.hashCode();
        i += 23 * (int)mCoords.latitude;
        i -= 19 * (int)mCoords.longitude;
        return i;
    }
}
